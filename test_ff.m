load map_2.mat;

% dbstop if error;
map = map_struct.map_samples{1};
global plan_map;
plan_map = dilate_map(map, map_struct.bridge_locations);

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

figure(1); clf;
imshow(imresize(plan_map, scale, 'nearest'),'Border','tight');
hold on;
plot(scale*map_struct.start.x,scale*map_struct.start.y,'g.', 'MarkerSize', 2*scale);
plot(scale*map_struct.goal.x,scale*map_struct.goal.y,'r.', 'MarkerSize', 2*scale);
axis equal;
axis([0 500 0 500]);
axis off;

s_start = [map_struct.start.x, map_struct.start.y, 0];
s_goal  = [map_struct.goal.x, map_struct.goal.y, 0];
% [~] = rrt(s_start, s_goal, map);
[~] = LPA(s_start, s_goal, plan_map);

hold off;