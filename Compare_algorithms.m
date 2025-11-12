% Run A*, RRT*, and PRM on the same scenario and compare performance
clc; clear; close all;

fprintf('\n=== Comparing A*, RRT*, and PRM in 3D Urban Scenario ===\n');

%% --- 1. Create environment
scenario = create_environment_3d();

%% --- 2. Run A*
fprintf('\nRunning A*...\n');
tic;
wps_astar = astar_3d_urban(scenario);
t_astar = toc;
L_astar = path_length(wps_astar);
fprintf('A*: time = %.2f s, path length = %.2f m\n', t_astar, L_astar);

%% --- 3. Run RRT*
fprintf('\nRunning RRT*...\n');
tic;
wps_rrt = rrt_star_3d_urban(scenario);
t_rrt = toc;
L_rrt = path_length(wps_rrt);
fprintf('RRT*: time = %.2f s, path length = %.2f m\n', t_rrt, L_rrt);

%% --- 4. Run PRM
fprintf('\nRunning PRM...\n');
tic;
wps_prm = prm_3d_urban(scenario);
t_prm = toc;
L_prm = path_length(wps_prm);
fprintf('PRM: time = %.2f s, path length = %.2f m\n', t_prm, L_prm);

%% --- 5. Show results together
figure('Position',[100 100 1200 800]); hold on; grid on; view(3);
xlim(scenario.x_lim); ylim(scenario.y_lim); zlim(scenario.z_lim);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Path Comparison: A*, RRT*, PRM');
axis equal

% Draw buildings
B = scenario.buildings;
for i = 1:size(B,1)
    draw_cuboid([B(i,1) B(i,2) B(i,3)], [B(i,4) B(i,5) B(i,6)], 0.3);
end

% Plot all paths
if ~isempty(wps_astar)
    plot3(wps_astar(:,1), wps_astar(:,2), wps_astar(:,3), 'b-o', 'LineWidth',1.8, 'DisplayName','A*');
end
if ~isempty(wps_rrt)
    plot3(wps_rrt(:,1), wps_rrt(:,2), wps_rrt(:,3), 'r--o', 'LineWidth',1.8, 'DisplayName','RRT*');
end
if ~isempty(wps_prm)
    plot3(wps_prm(:,1), wps_prm(:,2), wps_prm(:,3), 'g-.o', 'LineWidth',1.8, 'DisplayName','PRM');
end

plot3(scenario.start(1), scenario.start(2), scenario.start(3), 'k*', 'MarkerSize',10, 'DisplayName','Start');
plot3(scenario.goal(1),  scenario.goal(2),  scenario.goal(3),  'ko', 'MarkerSize',8,  'DisplayName','Goal');

lgd = legend('Location','northeastoutside');
lgd.ItemHitFcn = @(~,evt)set(evt.Peer,'Visible', ...
    iff(strcmp(evt.Peer.Visible,'on'),'off','on'));

%% --- 6. Summary table
fprintf('\n=== Summary ===\n');
fprintf('Algorithm   | Time (s) | Path Length (m)\n');
fprintf('----------------------------------------\n');
fprintf('A*          | %8.2f | %8.2f\n', t_astar, L_astar);
fprintf('RRT*        | %8.2f | %8.2f\n', t_rrt, L_rrt);
fprintf('PRM         | %8.2f | %8.2f\n', t_prm, L_prm);
fprintf('----------------------------------------\n');
fprintf('Done.\n');

%% --- Helper functions
function L = path_length(path)
    if isempty(path) || size(path,1) < 2
        L = 0; return;
    end
    diffs = diff(path);
    L = sum(sqrt(sum(diffs.^2,2)));
end

function draw_cuboid(origin, sizeXYZ, alpha)
    x = origin(1); y = origin(2); z = origin(3);
    w = sizeXYZ(1); d = sizeXYZ(2); h = sizeXYZ(3);
    V = [x y z;
         x+w y z;
         x+w y+d z;
         x y+d z;
         x y z+h;
         x+w y z+h;
         x+w y+d z+h;
         x y+d z+h];
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch('Vertices',V,'Faces',F,'FaceColor',[0.7 0.7 0.7], ...
          'FaceAlpha',alpha,'EdgeColor',[0.2 0.2 0.2]);
end

function out = iff(cond, a, b)
if cond, out = a; else, out = b; end
end