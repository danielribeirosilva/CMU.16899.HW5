function [scale] = LPA(s_start, s_goal, map)
% Input
%    - s_start
%    - s_goal
%    - map
%    
scale = 10;

% Build the heuristic map
h_map = floodfill(map, s_goal);

% max_steps = 1000000;
max_nodes = 1000000;

p_list = zeros(max_nodes, 1);  % parent list  pi(x) = parent(x)
u_list = zeros(max_nodes, 1);  % control list pi(x), u(x) -> x
s_list = zeros(max_nodes, 3);  % state list   s(x) = x, y, theta
g_list = inf*ones(max_nodes, 1);


pq = pq_create(max_nodes);

num_node = 1;
s_start(3)  = 0;
s_list(num_node,:) = s_start;
u_list(num_node)   = NaN;
p_list(num_node)   = 0;
g_list(num_node)   = 0;
h = interp2(h_map, s_start(1), s_start(2), 'linear');
pq_push(pq, num_node, -h);

goal_reached = false;
disp_counter = 1;
% while 
% 1. the list is not empty, 
% 2. the goal has not reached
% 3. the total cell expaned has not exceeded max_cells
while(pq_size(pq)>0 && num_node <= max_nodes)
   % Take the current best node
   [idx, cost] = pq_pop(pq);
   s_state = s_list(idx,:);
   s_g = g_list(idx);
   
   % Check if the goal is reached
   if ( isclose(s_state, s_goal))
      goal_reached = true;
      break;
   end
   
   % Expand this node: [0 1 -1 -2]
   [s_out, dg, flag] = reduced_ForwardMotion(s_state);
   if (flag)
      u_list(num_node) = 0;
      num_node = num_node+1; s_list(num_node,:) = s_out; p_list(num_node) = idx; g_list(num_node) = s_g + dg;
      h = interp2(h_map, s_out(1), s_out(2), 'linear');
      pq_push(pq, num_node, -g_list(num_node) - h);
   end
   
   [s_out, dg, flag] = reduced_LeftMotion(s_state);
   if (flag)
      u_list(num_node) = 1;
      num_node = num_node+1; s_list(num_node,:) = s_out; p_list(num_node) = idx; g_list(num_node) = s_g + dg;
      h = interp2(h_map, s_out(1), s_out(2), 'linear');
      pq_push(pq, num_node, -g_list(num_node) - h);
   end
   
   [s_out, dg, flag] = reduced_RightMotion(s_state);
   if (flag)
      u_list(num_node) = -1;
      num_node = num_node+1; s_list(num_node,:) = s_out; p_list(num_node) = idx; g_list(num_node) = s_g + dg;
      h = interp2(h_map, s_out(1), s_out(2), 'linear');
      pq_push(pq, num_node, -g_list(num_node) - h);
   end
   
   [s_out, dg, flag] = reduced_BackupMotion(s_state);
   if (flag)
      u_list(num_node) = -2;
      num_node = num_node+1; s_list(num_node,:) = s_out; p_list(num_node) = idx; g_list(num_node) = s_g + dg;
      h = interp2(h_map, s_out(1), s_out(2), 'linear');
      pq_push(pq, num_node, -g_list(num_node) - h);
   end
   
   % Debug show expansion:
   if (p_list(idx) >0)
      disp_counter = disp_counter+1;
      p_state = s_list(p_list(idx),:);
      line(scale*[p_state(1), s_state(1)], scale*[p_state(2), s_state(2)], 'Color','Red', 'LineSmoothing', 'on');
      if(mod(disp_counter, 100)==0)
         pause(0.05);
         fprintf('%d nodes grown\n', num_node);
      end
   end
end% end while

% If goal is reached, recover the path, else return error message
if (goal_reached)
   % return path
else
   % return error
end

pq_delete(pq);
end

function [flag] = isclose(s1, s2)
flag = norm([s1(1)-s2(1), s1(2)-s2(2)]) < 0.5;
end

function [safe] = checkNoCollision(state_out)
%*****************************
% Transform initial car polygon into referse frame of the car
%*****************************
global params;
global plan_map;
H = [cos(state_out(3)) -sin(state_out(3)) state_out(1);
               sin(state_out(3))  cos(state_out(3)) state_out(2);
               1                     1                    1          ;];
border = H*params.border;


%*****************************
% detect collisions
%*****************************    

[N, M] = size(plan_map);

%get (x,y) locations on the map 
[x,y] = meshgrid(1:N,1:M); 

% find all 0 locations on the map
[ind] = find(plan_map==0);

% check to see whether any of these locations falls inside our car
% polygon
in = inpolygon(x(ind), y(ind), border(1,:), border(2,:));

if (sum(in)>0) 
   safe = false;
else
   safe = true;
end
end

function [state_out, dg, flag] = reduced_RightMotion(state_in)
global params;

flag = 0; dg = 0;
for i=1:3
   r_dTheta = params.d_theta_nom + params.d_theta_max_dev;
   l_dTheta = params.d_theta_nom - params.d_theta_max_dev;
   
   R = 2*params.r_radius*r_dTheta;    % distance Right wheel traveled
   L = 2*params.l_radius*l_dTheta;    % distance Left wheel traveled
   
   state_out(1) = state_in(1) + params.wb/2*(R+L)/(R-L)*(sin((R-L)/params.wb + state_in(3)) - sin(state_in(3)));
   state_out(2) = state_in(2) - params.wb/2*(R+L)/(R-L)*(cos((R-L)/params.wb + state_in(3)) - cos(state_in(3)));
   state_out(3) = state_in(3) + (R-L)/params.wb;
   if(checkNoCollision(state_out))
      state_in = state_out; dg= dg+0.15; flag = 1;
   else
      state_out = state_in; return;
   end
end

end

function [state_out, dg, flag] = reduced_LeftMotion(state_in)
global params;

flag = 0; dg = 0;
for i=1:3
   r_dTheta = params.d_theta_nom - params.d_theta_max_dev;
   l_dTheta = params.d_theta_nom + params.d_theta_max_dev;
   
   R = 2*params.r_radius*r_dTheta;    % distance Right wheel traveled
   L = 2*params.l_radius*l_dTheta;    % distance Left wheel traveled
   
   state_out(1) = state_in(1) + params.wb/2*(R+L)/(R-L)*(sin((R-L)/params.wb + state_in(3)) - sin(state_in(3)));
   state_out(2) = state_in(2) - params.wb/2*(R+L)/(R-L)*(cos((R-L)/params.wb + state_in(3)) - cos(state_in(3)));
   state_out(3) = state_in(3) + (R-L)/params.wb;
   
   if(checkNoCollision(state_out))
      state_in = state_out; dg= dg+0.15; flag = 1;
   else
      state_out = state_in; return;
   end
end
end

function [state_out, dg, flag] = reduced_ForwardMotion(state_in)
global params;

flag = 0; dg = 0;
for i=1:3
   r_dTheta = params.d_theta_nom;
   l_dTheta = params.d_theta_nom;
   
   R = 2*params.r_radius*r_dTheta;    % distance Right wheel traveled
   L = 2*params.l_radius*l_dTheta;    % distance Left wheel traveled
   
   state_out(1) = state_in(1) + (R+L)/2*cos(state_in(3));
   state_out(2) = state_in(2) + (R+L)/2*sin(state_in(3));
   state_out(3) = state_in(3) + (R-L)/params.wb;
   
   if(checkNoCollision(state_out))
      state_in = state_out; dg= dg+0.15; flag = 1;
   else
      state_out = state_in; return;
   end
end

end

function [state_out, dg, flag] = reduced_BackupMotion(state_in)
global params;
    
flag = 0; dg = 0;
for i=1:3
   r_dTheta = -params.d_theta_reverse;
   l_dTheta = -params.d_theta_reverse;
   
   R = 2*params.r_radius*r_dTheta;    % distance Right wheel traveled
   L = 2*params.l_radius*l_dTheta;    % distance Left wheel traveled
   
   state_out(1) = state_in(1) + (R+L)/2*cos(state_in(3));
   state_out(2) = state_in(2) + (R+L)/2*sin(state_in(3));
   state_out(3) = state_in(3) + (R-L)/params.wb;
   
   if(checkNoCollision(state_out))
      state_in = state_out; dg= dg+0.15; flag = 1;
   else
      state_out = state_in; return;
   end
end
end

% function [s] = createNode(state, g, parent_idx, parent_u)
% s.state = state;
% s.g   = g;
% s.pi  = parent_idx;
% s.u   = parent_u;
% end
