function [ff] = floodfill(map, goal)
% A crude inplementation of breadth first floodfill as heuristic
% used for fine-grained A* serach
[M, N ] = size(map);

ff = inf*ones([M, N]);

ff(map == 0) = -1;

Q = {};
Q{1} = goal;
ff(goal(1), goal(2))= 0;

checkbr = @(r,c) r>0 && r<M && c>0 && c<N;

while(~isempty(Q))
    % The 8 neighbors' values are set
    elm = Q{1};
    Q = Q(2:end);
    
    value = ff(elm(1), elm(2));
    
    % Upper left
    r = elm(1)-1; c = elm(2)-1;
    if (checkbr(r, c))
       if (map(r, c)==1 && ff(r, c)>(value+1))
          ff(r, c) = value + 1;
          Q{end+1} = [r, c];
       end
    end
    
    % Up
    r = elm(1)-1; c = elm(2);
    if (checkbr(r, c))
       if (map(r, c)==1 && ff(r, c)>(value+1))
          ff(r, c) = value + 1;
          Q{end+1} = [r, c];
       end
    end
    
    % Upper Right
    r = elm(1)-1; c = elm(2)+1;
    if (checkbr(r, c))
       if (map(r, c)==1 && ff(r, c)>(value+1))
          ff(r, c) = value + 1;
          Q{end+1} = [r, c];
       end
    end
    
    % Left
    r = elm(1); c = elm(2)-1;
    if (checkbr(r, c))
       if (map(r, c)==1 && ff(r, c)>(value+1))
          ff(r, c) = value + 1;
          Q{end+1} = [r, c];
       end
    end
    
    % Right
    r = elm(1); c = elm(2)+1;
    if (checkbr(r, c))
       if (map(r, c)==1 && ff(r, c)>(value+1))
          ff(r, c) = value + 1;
          Q{end+1} = [r, c];
       end
    end
    
    % Lower left
    r = elm(1)+1; c = elm(2)-1;
    if (checkbr(r, c))
       if (map(r, c)==1 && ff(r, c)>(value+1))
          ff(r, c) = value + 1;
          Q{end+1} = [r, c];
       end
    end
    
    % Down
    r = elm(1)+1; c = elm(2);
    if (checkbr(r, c))
       if (map(r, c)==1 && ff(r, c)>(value+1))
          ff(r, c) = value + 1;
          Q{end+1} = [r, c];
       end
    end
    
    % Lowerright
    r = elm(1)+1; c = elm(2)+1;
    if (checkbr(r, c))
       if (map(r, c)==1 && ff(r, c)>(value+1))
          ff(r, c) = value + 1;
          Q{end+1} = [r, c];
       end
    end
end

end