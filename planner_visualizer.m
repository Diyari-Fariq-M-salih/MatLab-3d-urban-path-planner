function planner_visualizer(scenario, wps, algId, tNow, pathLen, planTime)
% Live 3-D visualization for the planners (called from Simulink).
% Maintains persistent figure/handles and updates efficiently.

persistent hFig hAx hPath hStart hGoal lastAlg

% Create figure & static environment once
if isempty(hFig) || ~isvalid(hFig)
    hFig = figure('Name','Urban Path Planning (live)','NumberTitle','off','Color','w');
    clf(hFig); hAx = axes('Parent',hFig); hold(hAx,'on'); grid(hAx,'on'); view(hAx,3);
    axis(hAx,'equal');
    xlim(hAx, scenario.x_lim); ylim(hAx, scenario.y_lim); zlim(hAx, scenario.z_lim);
    xlabel(hAx,'X (m)'); ylabel(hAx,'Y (m)'); zlabel(hAx,'Z (m)');

    % Draw buildings
    B = scenario.buildings;
    for i = 1:size(B,1)
        draw_cuboid(hAx, [B(i,1) B(i,2) B(i,3)], [B(i,4) B(i,5) B(i,6)], 0.25);
    end

    % Markers
    hStart = plot3(hAx, scenario.start(1), scenario.start(2), scenario.start(3), ...
        'k*', 'MarkerSize', 10, 'DisplayName', 'Start');
    hGoal  = plot3(hAx, scenario.goal(1),  scenario.goal(2),  scenario.goal(3),  ...
        'kp', 'MarkerSize', 10, 'MarkerFaceColor','r','DisplayName', 'Goal');

    % Path line (empty to start)
    hPath = plot3(hAx, nan, nan, nan, 'b-', 'LineWidth', 2, 'DisplayName', 'Path');
    legend(hAx,'Location','northeastoutside');
    lastAlg = algId;
end

% Update line data if path available
if ~isempty(wps) && size(wps,1)>=2 && isgraphics(hPath)
    set(hPath, 'XData', wps(:,1), 'YData', wps(:,2), 'ZData', wps(:,3));
else
    set(hPath, 'XData', nan, 'YData', nan, 'ZData', nan);
end

% Title with metrics and algorithm name
algNames = {'A*','RRT*','PRM'};
if algId < 1 || algId > 3, algId = 1; end
ttl = sprintf('Live: %s   |   t = %.1fs   |   len = %.1fm   |   plan = %.2fs', ...
    algNames{algId}, tNow, pathLen, planTime);
title(hAx, ttl);

drawnow limitrate
end


% ---------- helpers ----------
function draw_cuboid(ax, origin, sizeXYZ, alphaVal)
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
patch('Parent',ax,'Vertices',V,'Faces',F,'FaceColor',[0.7 0.7 0.7], ...
      'FaceAlpha',alphaVal,'EdgeColor',[0.2 0.2 0.2]);
end
