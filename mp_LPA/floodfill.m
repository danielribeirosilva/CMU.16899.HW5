function [h_map] = floodfill(map, goal)
% A crude inplementation of breadth first floodfill as heuristic
% used for fine-grained A* serach

% Function: check_obstacle
ob1 = @(r,c) numel(find(map(r:r+1, c:c+1)==0))==0;
ob2 = @(r,c) numel(find(map(r:r+1, c-1:c)==0))==0;
ob3 = @(r,c) numel(find(map(r-1:r, c:c+1)==0))==0;
ob4 = @(r,c) numel(find(map(r-1:r, c-1:c)==0))==0;

h_map = zeros([size(map), 4]);
h_map(:,:, 1) = mapfill(map, goal, ob1);
h_map(:,:, 2)  = mapfill(map, goal, ob2);
h_map(:,:, 3)  = mapfill(map, goal, ob3);
h_map(:,:, 4)  = mapfill(map, goal, ob4);
h_map = min(h_map, [], 3);
end

function [ff] = mapfill(map, goal, checkob)
[M, N ] = size(map);

ff = inf*ones([M, N]);

Q = {};

% ::Note: the order of y and x are flipped
Q{1} = [goal(2) goal(1)];
ff(goal(2), goal(1))= 0;

% Function: check_boarder
checkbr = @(r,c) r>0 && r<M && c>0 && c<N;

while(~isempty(Q))
    % The 8 neighbors' values are set
    elm = Q{1};
    Q = Q(2:end);
    
    value = ff(elm(1), elm(2));
    
    % Upper left
    r = elm(1)-1; c = elm(2)-1;
    if (checkbr(r, c) && checkob(r,c))
       if (map(r, c)>0 && ff(r, c)>(value+sqrt(2)))
          ff(r, c) = value + sqrt(2);
          Q{end+1} = [r, c];
       end
    end
    
    % Up
    r = elm(1)-1; c = elm(2);
    if (checkbr(r, c) && checkob(r,c))
       if (map(r, c)>0 && ff(r, c)>(value+1))
          ff(r, c) = value + 1;
          Q{end+1} = [r, c];
       end
    end
    
    % Upper Right
    r = elm(1)-1; c = elm(2)+1;
    if (checkbr(r, c) && checkob(r,c))
       if (map(r, c)>0 && ff(r, c)>(value+sqrt(2)))
          ff(r, c) = value + sqrt(2);
          Q{end+1} = [r, c];
       end
    end
    
    % Left
    r = elm(1); c = elm(2)-1;
    if (checkbr(r, c) && checkob(r,c))
       if (map(r, c)>0 && ff(r, c)>(value+1))
          ff(r, c) = value + 1;
          Q{end+1} = [r, c];
       end
    end
    
    % Right
    r = elm(1); c = elm(2)+1;
    if (checkbr(r, c) && checkob(r,c))
       if (map(r, c)>0 && ff(r, c)>(value+1))
          ff(r, c) = value + 1;
          Q{end+1} = [r, c];
       end
    end
    
    % Lower left
    r = elm(1)+1; c = elm(2)-1;
    if (checkbr(r, c) && checkob(r,c))
       if (map(r, c)>0 && ff(r, c)>(value+sqrt(2)))
          ff(r, c) = value + sqrt(2);
          Q{end+1} = [r, c];
       end
    end
    
    % Down
    r = elm(1)+1; c = elm(2);
    if (checkbr(r, c) && checkob(r,c))
       if (map(r, c)>0 && ff(r, c)>(value+1))
          ff(r, c) = value + 1;
          Q{end+1} = [r, c];
       end
    end
    
    % Lowerright
    r = elm(1)+1; c = elm(2)+1;
    if (checkbr(r, c) && checkob(r,c))
       if (map(r, c)>0 && ff(r, c)>(value+sqrt(2)))
          ff(r, c) = value + sqrt(2);
          Q{end+1} = [r, c];
       end
    end
end
end