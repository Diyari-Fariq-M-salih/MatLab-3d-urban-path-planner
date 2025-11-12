function waypoints = planification_globale(scenario)
% use after creating environment to select algorithm methods
% 1/2/3 = A*/rrt*/prm

fprintf('\n=== Algorithm Selection ===\n');
fprintf(' 1) A* (Optimal on grid; potentially expensive)\n');
fprintf(' 2) RRT* (Fast; asymptotically optimal)\n');
fprintf(' 3) PRM 3D (Pre-computed / reusable graph)\n');
choice = input('Select algorithm (1-3): ');

switch choice
    case 1
        waypoints = astar_3d_urban(scenario);
    case 2
        waypoints = rrt_star_3d_urban(scenario);
    case 3
        waypoints = prm_3d_urban(scenario);
    otherwise
        warning('Unknown choice. Falling back to A*.');
        waypoints = astar_3d_urban(scenario);
end
end
