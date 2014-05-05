function [dilated_map] = dilate_map(map, bridges)

dilated_map = map;

num_bridges = size(bridges, 2);
for i=1:num_bridges
   br = bridges(2, i);
   bc = bridges(1, i);
   if (map(br, bc) == 0)
      dilated_map(br-1:br+1, bc-1:bc+1) = 0;
   end
end

end