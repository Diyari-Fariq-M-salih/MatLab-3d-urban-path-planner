function waypoints = prm_3d_urban(scenario)
%Adaptive 3D Probabilistic Roadmap (PRM) planner
% INPUT:
%   scenario : environment struct from create_environment_3d()
% OUTPUT:
%   waypoints : Nx3 matrix of path points [x y z]

%% Parameters
maxTries     = 5;      % Maximum resampling attempts
numSamples   = 1000;   % Initial number of random nodes
kNeighbors   = 30;     % Number of neighbor connections
resolution   = 4;      % Grid resolution for collision checks (meters)

fprintf('Building PRM roadmap...\n');

%% Build Occupancy Grid
xg = scenario.x_lim(1):resolution:scenario.x_lim(2);
yg = scenario.y_lim(1):resolution:scenario.y_lim(2);
zg = max(scenario.altitude_min, scenario.z_lim(1)) : resolution : ...
     min(scenario.altitude_max, scenario.z_lim(2));
occ = calculate_cost_urban(xg, yg, zg, scenario);

isFree = @(p) ~check_collision(p, xg, yg, zg, occ);
start = scenario.start;
goal  = scenario.goal;

if ~isFree(start) || ~isFree(goal)
    error('PRM: start or goal inside obstacle or inflated zone');
end

found = false;

%% Adaptive Sampling Loop
for attempt = 1:maxTries
    fprintf('  Attempt %d/%d...\n', attempt, maxTries);

    % --- 1) Sample free points in the map
    samples = [];
    while size(samples,1) < numSamples
        p = [rand_range(scenario.x_lim), ...
             rand_range(scenario.y_lim), ...
             rand_range([scenario.altitude_min scenario.altitude_max])];
        if isFree(p)
            samples(end+1,:) = p; %#ok<AGROW>
        end
    end

    % --- 2) Include start & goal
    V = [start; samples; goal];
    N = size(V,1);

    % --- 3) Build adjacency matrix
    A = inf(N); % adjacency matrix (symmetric)
    for i = 1:N
        dists = vecnorm(V - V(i,:), 2, 2);
        [~, idx] = sort(dists);
        for j = idx(2:min(kNeighbors+1, N))'
            if edge_is_free(V(i,:), V(j,:), xg, yg, zg, occ, resolution)
                A(i,j) = dists(j);
                A(j,i) = dists(j);
            end
        end
    end

    fprintf('  Checking connectivity...\n');
    numEdges = sum(isfinite(A(:)))/2;
    fprintf('  Nodes: %d  Edges: %d\n', N, numEdges);

    % --- 4) Use MATLAB's fast graph shortest path
    fprintf('  Solving shortest path with built-in graph()...\n');
    A(isinf(A)) = 0;        % convert to zero for sparse representation
    G = graph(A, 'upper');  % create sparse graph

    [pathIdx, pathDist] = shortestpath(G, 1, N, 'Method', 'positive');

    % --- 5) Check result ------------------------------------------------
    if isempty(pathIdx)
        fprintf('    No connection found — resampling with +%d nodes\n', 500);
        numSamples = numSamples + 500;  % densify
    else
        found = true;
        break;
    end
end

%% Output
if ~found
    warning('PRM: No valid path found even after %d attempts — using A* fallback.', maxTries);
    waypoints = astar_3d_urban(scenario);
else
    fprintf('  ✓ PRM path found with %d waypoints, total distance = %.2f m\n', ...
            numel(pathIdx), pathDist);
    waypoints = V(pathIdx,:);
end

end

% Helper functions
function v = rand_range(lim)
v = lim(1) + rand*(lim(2)-lim(1));
end

function free = edge_is_free(p1, p2, xg, yg, zg, occ, step)
L = norm(p2-p1);
n = max(2, ceil(L/step));
pts = [linspace(p1(1),p2(1),n)', linspace(p1(2),p2(2),n)', linspace(p1(3),p2(3),n)'];
free = true;
for i = 1:n
    if check_collision(pts(i,:), xg, yg, zg, occ)
        free = false; return
    end
end
end

function c = check_collision(p, xg, yg, zg, occ)
[~,ix] = min(abs(xg - p(1)));
[~,iy] = min(abs(yg - p(2)));
[~,iz] = min(abs(zg - p(3)));
ix = max(min(ix, numel(xg)),1);
iy = max(min(iy, numel(yg)),1);
iz = max(min(iz, numel(zg)),1);
c = occ(ix,iy,iz);
end
