function env = create_environment_3d()
%CREATE_ENVIRONMENT_3D  Define a simple 3-D urban scenario
%Returns a struct 'env' used by the planners/visualizer.

% ---- Mission / world limits
env.mission = 'Urban Surveillance Mission';
env.x_lim   = [0 100];
env.y_lim   = [0 100];
env.z_lim   = [0  50];

% ---- Buildings  [x  y  z  width  depth  height]
env.buildings = [ ...
    20 30  0  25 25 35;   % B1
    60 20  0  20 30 25;   % B2
    10 70  0  40 20 35;   % B3
    70 70  0  25 20 45;   % B4
];

%extra obstacles centers + radius (spheres)(unused)
env.obstacles_spheres = [ ...
%   xc  yc  zc  r
];

% ---- Start / goal 
env.start = [5 35 20];
env.goal  = [95 30 40];

%(optional) manual intermediate waypoints
env.waypoints_intermediates = [ ...
%  x   y   z 
];

% ---- Drone constraints
env.altitude_min     = 10;
env.altitude_max     = 40;
env.vmax             = 15;    % m/s 
env.turning_radius_m = 10;    % m   
end
