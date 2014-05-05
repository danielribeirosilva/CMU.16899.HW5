% Hw 5 Minghao Ruan
% Main test script

%% If you are running the code the first time, uncomment the following two
%% line to add path and build the priority functions
% addpath(genpath('astar'));
% run('astar/myheaps/src/makefile.m'); % build the priority queue mex files

%% Then run the code 
%%
clear
% Some hard test cases:
% 2_1
% 3_20
% 5_1
load('all_maps/map_5.mat'); % Load map
map = map_struct.map_samples{1}; % which test map

DISPLAY_TYPE = 0;
load_sim_params;
initialize_state;
[N,M] = size(map_struct.seed_map);
[x,y] = meshgrid(1:N, 1:M);

global scale;
scale = 10;

display_environment

%% Plan with the initial map
s_start = [map_struct.start.x, map_struct.start.y, 0];
s_goal  = [map_struct.goal.x, map_struct.goal.y, 0];
% [~] = rrt(s_start, s_goal, map);
[path, flag] = aastar(s_start, s_goal, map_struct.seed_map, params, 5);

if (~flag)
    error('No path found\n');
end

%% Run the path, replan if necessary
i=1;
while (i<numel(path))
   
   action = path(i);
   [state, updated_map, flags] = motionModel(params, state, action, observed_map, map, goal);
   
   display_environment;
   if (flags == 1 || flags ==2)
      break;
   end
   
   idx = find(updated_map~= observed_map);
   observed_map = updated_map;
   % Only replan if we see a bridge is actually blocked.
   if ( any(updated_map(idx)==0))
       fprintf('Map updates, replan!\n');
       display_environment;
       % Replan:
       s_start = [state.x state.y state.theta];
       [path, flag] = aastar(s_start, s_goal, observed_map, params, 5);
       if (~flag)
           error('No path found\n');
       else
           i=0;
       end
   end
   i= i+1;
end

hold off;