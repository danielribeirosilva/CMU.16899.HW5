load map_1.mat;

map = map_struct.map_samples{2};

tic
ff = floodfill(map, map_struct.goal);
toc