clear
load('all_maps/map_3.mat');
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
[path, flag] = aastar(s_start, s_goal, map_struct.seed_map, params, 5);

if (~flag)
    error('No path found\n');
end

i=1;
while (i<numel(path))
   
   action = path(i);
   [state, updated_map, flags] = motionModel(params, state, action, observed_map, map, goal);
   
%    plot(scale*state.x,scale*state.y,'b.', 'MarkerSize', 2*scale);
%    line(scale*[state.x,state.x+params.length/2*cos(state.theta)]',scale*[state.y,state.y+params.length/2*sin(state.theta)]','Color','Blue', 'LineSmoothing', 'on');

   display_environment;
   if (flags == 1 || flags ==2)
      break;
   end
   
   if (~isequal(updated_map, observed_map))
       fprintf('Map updates, replan!\n');
       display_environment;
       % Replan:
       observed_map = updated_map;
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