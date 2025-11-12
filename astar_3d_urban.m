function waypoints = astar_3d_urban(scenario)
%3-D A* on a voxel grid with building obstacles

% Grid resolution (m)
resolution = 5;

% Discretize world
x_grid = scenario.x_lim(1):resolution:scenario.x_lim(2);
y_grid = scenario.y_lim(1):resolution:scenario.y_lim(2);
z_grid = max(scenario.altitude_min, scenario.z_lim(1)) : resolution : ...
         min(scenario.altitude_max, scenario.z_lim(2));

% Occupancy / cost map
occ = calculate_cost_urban(x_grid, y_grid, z_grid, scenario);  % logical occupancy
if all(occ(:))
    error('Environment fully blocked at this resolution.');
end

%Helper indexers
nx = numel(x_grid); ny = numel(y_grid); nz = numel(z_grid);
toIdx  = @(ix,iy,iz) sub2ind([nx ny nz], ix,iy,iz);
toSub  = @(idx) ind2sub_vec([nx ny nz], idx);

% ---- World->Grid index conversion
s_ijk = world_to_grid(scenario.start, x_grid, y_grid, z_grid);
g_ijk = world_to_grid(scenario.goal,  x_grid, y_grid, z_grid);

% guard limits
if any(s_ijk<1) || any(g_ijk<1) || ...
   s_ijk(1)>nx || s_ijk(2)>ny || s_ijk(3)>nz || ...
   g_ijk(1)>nx || g_ijk(2)>ny || g_ijk(3)>nz
    error('Start/Goal outside grid or altitude bounds.');
end

s = toIdx(s_ijk(1), s_ijk(2), s_ijk(3));
g = toIdx(g_ijk(1), g_ijk(2), g_ijk(3));
if occ(s) || occ(g)
    error('Start or goal cell is occupied.');
end

% A* setup
gScore = inf(nx*ny*nz,1);
fScore = inf(nx*ny*nz,1);
cameFrom = zeros(nx*ny*nz,1,'uint32');
open = false(nx*ny*nz,1);
closed = false(nx*ny*nz,1);

gScore(s) = 0;
fScore(s) = heuristic_cost_estimate(s, g, toSub, x_grid, y_grid, z_grid);
open(s)   = true;

% 26-connected neighborhood (dx,dy,dz in -1..1 except 0,0,0)
[DX,DY,DZ] = ndgrid(-1:1,-1:1,-1:1);
nbrs = [DX(:) DY(:) DZ(:)];
nbrs(~any(nbrs,2),:) = [];  % remove [0 0 0]

% ---- Main loop
while any(open)
    % pick node with smallest fScore in OPEN
    openIdx = find(open);
    [~,k]   = min(fScore(openIdx));
    current = openIdx(k);

    if current==g
        % reconstruct
        path_idx = reconstruct_path(cameFrom, current);
        waypoints = grid_path_to_world(path_idx, toSub, x_grid, y_grid, z_grid);
        waypoints = unique(waypoints, 'rows', 'stable');
        return
    end

    open(current)   = false;
    closed(current) = true;

    [ci, cj, ck] = toSub(current);

    % expand neighbors
    for q = 1:size(nbrs,1)
        ni = ci + nbrs(q,1);
        nj = cj + nbrs(q,2);
        nk = ck + nbrs(q,3);

        if ni<1 || nj<1 || nk<1 || ni>nx || nj>ny || nk>nz
            continue
        end

        nid = toIdx(ni,nj,nk);
        if closed(nid) || occ(nid)
            continue
        end

        step = sqrt((nbrs(q,1)*resolution)^2 + ...
                    (nbrs(q,2)*resolution)^2 + ...
                    (nbrs(q,3)*resolution)^2);

        tentative_g = gScore(current) + step;

        if ~open(nid)
            open(nid) = true;
        elseif tentative_g >= gScore(nid)
            continue
        end

        cameFrom(nid) = current;
        gScore(nid)   = tentative_g;
        fScore(nid)   = tentative_g + ...
            heuristic_cost_estimate(nid, g, toSub, x_grid, y_grid, z_grid);
    end
end

error('A* failed: no path found (try coarser resolution or edit buildings).');
end

% helpers
function occ = calculate_cost_urban(xg, yg, zg, scenario)
% Occ: true = blocked. Buildings are axis-aligned cuboids.
[nx,ny,nz] = deal(numel(xg), numel(yg), numel(zg));
occ = false(nx,ny,nz);

% mark buildings
B = scenario.buildings;
for i = 1:size(B,1)
    x0=B(i,1); y0=B(i,2); z0=B(i,3);
    w =B(i,4); d =B(i,5); h =B(i,6);

    x1 = x0;      x2 = x0 + w;
    y1 = y0;      y2 = y0 + d;
    z1 = z0;      z2 = z0 + h;

    xi = find(xg>=x1 & xg<=x2);
    yi = find(yg>=y1 & yg<=y2);
    zi = find(zg>=z1 & zg<=z2);

    if ~isempty(xi) && ~isempty(yi) && ~isempty(zi)
        occ(xi, yi, zi) = true;
    end
end

end

function h = heuristic_cost_estimate(a, g, toSub, xg, yg, zg)
[ai,aj,ak] = toSub(a);
[gi,gj,gk] = toSub(g);
dx = xg(ai) - xg(gi);
dy = yg(aj) - yg(gj);
dz = zg(ak) - zg(gk);
h = sqrt(dx*dx + dy*dy + dz*dz);
end

function p = reconstruct_path(cameFrom, current)
p = current;
while cameFrom(current)~=0
    current = cameFrom(current);
    p = [current; p]; %#ok<AGROW>
end
end

function xyz = grid_path_to_world(idx_path, toSub, xg, yg, zg)
xyz = zeros(numel(idx_path),3);
for i=1:numel(idx_path)
    [ii,jj,kk] = toSub(idx_path(i));
    xyz(i,:) = [xg(ii), yg(jj), zg(kk)];
end
end

function ijk = world_to_grid(p, xg, yg, zg)
[~,ix] = min(abs(xg - p(1)));
[~,iy] = min(abs(yg - p(2)));
[~,iz] = min(abs(zg - p(3)));
ijk = [ix iy iz];
end

function varargout = ind2sub_vec(sz, ind)
[i,j,k] = ind2sub(sz, ind);
varargout = {i,j,k};
end
