function visualize_mission(scenario, waypoints)
%Render buildings + the planned 3-D path

figure('Position',[100 100 1200 800]);
subplot(2,2,[1 3]); hold on; grid on; box on; view(3);

xlim(scenario.x_lim); ylim(scenario.y_lim); zlim(scenario.z_lim);

%Draw buildings
B = scenario.buildings;
for i = 1:size(B,1)
    draw_cuboid([B(i,1) B(i,2) B(i,3)], [B(i,4) B(i,5) B(i,6)], 0.4);
end

% Plot path + markers
if ~isempty(waypoints)
    plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'b-o', ...
        'LineWidth',1.8, 'MarkerSize',4, 'DisplayName','Planned Path');
end
plot3(scenario.start(1), scenario.start(2), scenario.start(3), 'g*', ...
    'MarkerSize',10, 'DisplayName','Start');
plot3(scenario.goal(1), scenario.goal(2), scenario.goal(3), 'ro', ...
    'MarkerSize',7, 'DisplayName','Goal');

xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('Location','northeastoutside'); axis equal
title(scenario.mission);
end

% local helper
function draw_cuboid(origin, sizeXYZ, faceAlpha)
% origin = [x y z] of minimum corner; sizeXYZ = [w d h]
x = origin(1); y = origin(2); z = origin(3);
w = sizeXYZ(4-3); d = sizeXYZ(5-3); h = sizeXYZ(6-3);
% vertices
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
      'FaceAlpha',faceAlpha,'EdgeColor',[0.2 0.2 0.2]);
end
