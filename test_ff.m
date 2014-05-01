load map_1.mat;

map = map_struct.map_samples{1};

goal = [map_struct.goal.y, map_struct.goal.x];

ff = floodfill(map, goal);