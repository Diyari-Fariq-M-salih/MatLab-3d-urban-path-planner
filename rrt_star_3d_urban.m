function waypoints = rrt_star_3d_urban(scenario)
% 3D RRT* implementation
% Returns a list of [x y z] waypoints from start to goal.

rng('shuffle');

% Parameters
maxNodes     = 2000;   % maximum tree size
goalRadius   = 5;      % goal capture radius (m)
neighborDist = 15;     % radius for rewiring (m)
stepSize     = 5;      % incremental step length (m)
resolution   = 5;      

% Grid for collision checking
xg = scenario.x_lim(1):resolution:scenario.x_lim(2);
yg = scenario.y_lim(1):resolution:scenario.y_lim(2);
zg = max(scenario.altitude_min, scenario.z_lim(1)) : resolution : ...
     min(scenario.altitude_max, scenario.z_lim(2));
occ = calculate_cost_urban(xg, yg, zg, scenario);

% helper
isFree = @(p) ~check_collision(p, xg, yg, zg, occ);

% Start and goal
start = scenario.start;
goal  = scenario.goal;

if ~isFree(start)
    error('RRT*: start inside obstacle');
end
if ~isFree(goal)
    error('RRT*: goal inside obstacle');
end

% Initialize tree
nodes(1).pos = start;
nodes(1).parent = 0;
nodes(1).cost = 0;

for iter = 1:maxNodes
    % Random sample
    if rand < 0.1
        sample = goal; % goal bias
    else
        sample = [rand_range(scenario.x_lim);
                  rand_range(scenario.y_lim);
                  rand_range([scenario.altitude_min scenario.altitude_max])]';
    end

    % Find nearest node
    dists = cellfun(@(p) norm(p - sample), {nodes.pos});
    [~, idxNear] = min(dists);
    qNear = nodes(idxNear).pos;

    % Steer
    dir = sample - qNear;
    dist = norm(dir);
    if dist == 0, continue; end
    qNew = qNear + stepSize * dir/dist;

    % Check collision
    if ~edge_is_free(qNear, qNew, xg, yg, zg, occ, stepSize/2)
        continue
    end

    % Cost to new node
    cNew = nodes(idxNear).cost + norm(qNew - qNear);

    % Add node
    nodes(end+1).pos = qNew;
    nodes(end).parent = idxNear;
    nodes(end).cost = cNew;

    % Rewire nearby nodes (RRT*)
    idxNeighbors = find(cellfun(@(p) norm(p - qNew) < neighborDist, {nodes.pos}));
    for k = idxNeighbors
        if k == numel(nodes), continue; end
        if edge_is_free(nodes(k).pos, qNew, xg, yg, zg, occ, stepSize/2)
            newCost = nodes(k).cost + norm(nodes(k).pos - qNew);
            if newCost < nodes(end).cost
                nodes(end).parent = k;
                nodes(end).cost = newCost;
            elseif nodes(end).cost + norm(nodes(k).pos - qNew) < nodes(k).cost
                nodes(k).parent = numel(nodes);
                nodes(k).cost = nodes(end).cost + norm(nodes(k).pos - qNew);
            end
        end
    end

    % Goal check
    if norm(qNew - goal) < goalRadius && edge_is_free(qNew, goal, xg, yg, zg, occ, stepSize/2)
        % Add goal as node
        nodes(end+1).pos = goal;
        nodes(end).parent = numel(nodes)-1;
        break
    end
end

% Reconstruct path
if norm(nodes(end).pos - goal) > goalRadius
    warning('RRT*: goal not reached. Using closest node.');
    [~, idxMin] = min(cellfun(@(p) norm(p - goal), {nodes.pos}));
    q = idxMin;
else
    q = numel(nodes);
end

path = [];
while q ~= 0
    path = [nodes(q).pos; path]; %#ok<AGROW>
    q = nodes(q).parent;
end

waypoints = path;
end

% helper function
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
ix=max(ix,1); iy=max(iy,1); iz=max(iz,1);
ix=min(ix,numel(xg)); iy=min(iy,numel(yg)); iz=min(iz,numel(zg));
c = occ(ix,iy,iz);
end
