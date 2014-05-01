load map_1.mat;

dbstop if error;
map = map_struct.map_samples{2};
global plan_map;
plan_map = map;

% tic
% %ff = floodfill(map, map_struct.goal);
% toc

load_sim_params;

% scale is used to blow up the environment for display purposes only. Set
% to whatever looks good on your screen
scale = 10;

% determines the size of the map and creates a meshgrid for display
% purposes
[N,M] = size(map_struct.seed_map);
[x,y] = meshgrid(1:N, 1:M);

DISPLAY_ON = 1; % 1 - turns display on, 0 - turns display off
DISPLAY_TYPE = 1; % 0 - displays map as dots, 1 - displays map as blocks

imshow(imresize(map, scale, 'nearest'),'Border','tight');
plot(scale*map_struct.start.x,scale*map_struct.start.y,'g.', 'MarkerSize', 2*scale);
plot(scale*map_struct.goal.x,scale*map_struct.goal.y,'r.', 'MarkerSize', 2*scale);
axis equal;
axis([0 500 0 500]);
axis off;

hold on;
s_start = [map_struct.start.x, map_struct.start.y, 0];
s_goal  = [map_struct.goal.x, map_struct.goal.y, 0];
[~] = rrt(s_start, s_goal, map);
%[~] = LPA(s_start, s_goal, map);

hold off;