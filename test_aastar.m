clear all;
load map_3.mat;
% Test cases:
% 3_20
% 1_1
% 2_1

% dbstop if error;
map = map_struct.map_samples{20};

DISPLAY_TYPE = 0;
load_sim_params;
initialize_state;
[N,M] = size(map_struct.seed_map);
[x,y] = meshgrid(1:N, 1:M);
scale = 10;

display_environment

s_start = [map_struct.start.x, map_struct.start.y, 0];
s_goal  = [map_struct.goal.x, map_struct.goal.y, 0];
% [~] = rrt(s_start, s_goal, map);
[path, flag] = aastar(s_start, s_goal, map, params, 5);