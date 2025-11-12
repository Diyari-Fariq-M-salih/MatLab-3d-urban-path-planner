clear; close all; clc

scenario = create_environment_3d();

algorithms = {@astar_3d_urban, @rrt_star_3d_urban, @prm_3d_urban};
names = {'A*','RRT*','PRM'};

figure('Color','w'); hold on; grid on; view(3)
xlim(scenario.x_lim); ylim(scenario.y_lim); zlim(scenario.z_lim);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D Path Planning Comparison'); axis equal

for i = 1:numel(algorithms)
    t0 = tic;
    wps = algorithms{i}(scenario);
    t = toc(t0);
    if isempty(wps), continue; end
    d = diff(wps);
    len = sum(sqrt(sum(d.^2,2)));
    plot3(wps(:,1),wps(:,2),wps(:,3),'LineWidth',2, ...
          'DisplayName',sprintf('%s | %.2fs | %.1fm',names{i},t,len));
end

plot3(scenario.start(1),scenario.start(2),scenario.start(3),'ko','MarkerFaceColor','g','DisplayName','Start')
plot3(scenario.goal(1),scenario.goal(2),scenario.goal(3),'kp','MarkerFaceColor','r','DisplayName','Goal')
legend('Location','northeastoutside')
