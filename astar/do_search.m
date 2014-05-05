function [u_path, state_out, flag] = do_search(s_start, s_goal, map, params)
% Input
%    - s_start
%    - s_goal
%    - map
%    
global scale;
age_threshold = 5;
max_nodes = 100000;

% Build the heuristic map
h_map = floodfill(map, s_goal);

%% Basic data structures
p_list = zeros(max_nodes, 1);  % parent list  pi(x) = parent(x)
u_list = zeros(max_nodes, 1);  % control list pi(x), u(x) -> x
s_list = zeros(max_nodes, 3);  % state list   s(x) = x, y, theta
g_list = zeros(max_nodes, 1);
c_list = zeros(max_nodes, 1);
a_list = zeros(max_nodes, 1);  % age list

% Debug lists
h_list = inf*ones(max_nodes, 1);
f_list = inf*ones(max_nodes, 1);

% get (x,y) locations on the map that are needed for collision checking
% (avoid recomputing to speed up the check)
[x,y] = meshgrid(1:size(map, 1),1:size(map, 2));
% find all 0 locations on the map
[ind] = find(map==0);
x = x(ind); y = y(ind);
      
pq = pq_create(max_nodes);

num_node = 1;
%s_start(3)  = 0;
s_list(num_node,:) = s_start;
u_list(num_node)   = NaN;
p_list(num_node)   = 0;
g_list(num_node)   = 0;
a_list(num_node)   = 0;
h = interp2(h_map, s_start(1), s_start(2), 'linear');
h_list(num_node) = h; f_list(num_node) = g_list(num_node)+h;
pq_push(pq, num_node, -h);

goal_reached = false;
disp_counter = 1;
cur_max_age = 0;

%% while 
% 1. the list is not empty, 
% 2. the goal has not reached
% 3. the total cell expaned has not exceeded max_cells
while(pq_size(pq)>0 && num_node < max_nodes-3)
   % Take the current best node
   [idx, ~] = pq_pop(pq);
   s_state = s_list(idx,:);
   s_g = g_list(idx);
   
   % Check if the goal is reached
   if (norm([s_state(1)-s_goal(1), s_state(2)-s_goal(2)]) < 0.5)
      goal_reached = true;
      break;
   end
   
   if ( (cur_max_age - a_list(idx)) > age_threshold)
      continue;
   end % end of age check

   % Expand this node: [0 1 -1 -2]
    if(u_list(idx) ~= -2)
      [s_out, dg, flag] = reduced_ForwardMotion(s_state);
      try_add_new_node(0, flag);
    end
   
    if (u_list(idx) ~= 1)
      [s_out, dg, flag] = reduced_LeftMotion(s_state);
      try_add_new_node(-1, flag);
    end
   
    if (u_list(idx) ~= -1)
      [s_out, dg, flag] = reduced_RightMotion(s_state);
      try_add_new_node(1, flag);
    end
   
    if(u_list(idx) ~= 0)
      [s_out, dg, flag] = reduced_BackupMotion(s_state);
      try_add_new_node(-2, flag);
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

% Free the memory
pq_delete(pq);

%%
% If goal is reached, recover the path, 
% else return a path to max distant state
if (goal_reached)
   cidx = idx;  flag = true;
   pause(0.05);
   fprintf('%d nodes grown, goal reached.\n', num_node);
else
   [~, cidx] = max(g_list); flag = false; 
end
state_out = s_list(cidx,:);

% First unwind the sequence to count number of steps
count = 0;
pidx = cidx;
while(p_list(pidx) >0)
   count = count + c_list(pidx);
   pidx = p_list(pidx);
end
u_path = zeros(count,1);

% Add the path from the end
pidx = cidx;
base = count;
while(p_list(pidx) >0)
   u_path(base : -1 : base-c_list(pidx)+1) = u_list(pidx);
   base = base-c_list(pidx);
   pidx = p_list(pidx);
end

%% End of Code Section
% On to functions
   function [] = try_add_new_node(u, c)
      if (flag < 3)
          return
      end
      h = interp2(h_map, s_out(1), s_out(2), 'linear'); 
      if (isinf(h))
         return;
      end
      num_node = num_node+1;
      u_list(num_node) = u; 
      c_list(num_node) = c;
      s_list(num_node,:) = s_out; 
      p_list(num_node) = idx; 
      g_list(num_node) = s_g + dg;
      pq_push(pq, num_node, -g_list(num_node) - h);
      h_list(num_node) = h; 
      f_list(num_node) = g_list(num_node)+h;
      a_list(num_node) = a_list(idx) + 1; cur_max_age = max([cur_max_age, a_list(num_node)]);
   end

   function [state_out, dg, c] = reduced_RightMotion(state_in)
      state_out = state_in;
      c = 0; dg = 0;
      for i=1:9
         r_dTheta = params.d_theta_nom + params.d_theta_max_dev;
         l_dTheta = params.d_theta_nom - params.d_theta_max_dev;
         
         R = params.r_radius*r_dTheta;    % distance Right wheel traveled
         L = params.l_radius*l_dTheta;    % distance Left wheel traveled
         
         state_out(1) = state_in(1) + params.wb/2*(R+L)/(R-L)*(sin((R-L)/params.wb + state_in(3)) - sin(state_in(3)));
         state_out(2) = state_in(2) - params.wb/2*(R+L)/(R-L)*(cos((R-L)/params.wb + state_in(3)) - cos(state_in(3)));
         state_out(3) = state_in(3) + (R-L)/params.wb;
         
         if (state_out(3) > pi)
            state_out(3) = state_out(3) - 2*pi;
         elseif(state_out(3) < -pi)
            state_out(3) = state_out(3) + 2*pi;
         end
         
         if(checkNoCollision(state_out))
            state_in = state_out; dg= dg+0.15; c = c+1;
         else
            state_out = state_in; return;
         end
      end
      
   end

   function [state_out, dg, c] = reduced_LeftMotion(state_in)

      state_out = state_in;
      c = 0; dg = 0;
      for i=1:9
         r_dTheta = params.d_theta_nom - params.d_theta_max_dev;
         l_dTheta = params.d_theta_nom + params.d_theta_max_dev;
         
         R = params.r_radius*r_dTheta;    % distance Right wheel traveled
         L = params.l_radius*l_dTheta;    % distance Left wheel traveled
         
         state_out(1) = state_in(1) + params.wb/2*(R+L)/(R-L)*(sin((R-L)/params.wb + state_in(3)) - sin(state_in(3)));
         state_out(2) = state_in(2) - params.wb/2*(R+L)/(R-L)*(cos((R-L)/params.wb + state_in(3)) - cos(state_in(3)));
         state_out(3) = state_in(3) + (R-L)/params.wb;
         if (state_out(3) > pi)
            state_out(3) = state_out(3) - 2*pi;
         elseif(state_out(3) < -pi)
            state_out(3) = state_out(3) + 2*pi;
         end
         
         if(checkNoCollision(state_out))
            state_in = state_out; dg= dg+0.15; c = c+1;
         else
            state_out = state_in; return;
         end
      end
   end

   function [state_out, dg, c] = reduced_ForwardMotion(state_in)
      
      state_out = state_in;
      c = 0; dg = 0;
      for i=1:9
         r_dTheta = params.d_theta_nom;
         l_dTheta = params.d_theta_nom;
         
         R = params.r_radius*r_dTheta;    % distance Right wheel traveled
         L = params.l_radius*l_dTheta;    % distance Left wheel traveled
         
         state_out(1) = state_in(1) + (R+L)/2*cos(state_in(3));
         state_out(2) = state_in(2) + (R+L)/2*sin(state_in(3));
         state_out(3) = state_in(3);
         
         if(checkNoCollision(state_out))
            state_in = state_out; dg= dg+0.15; c = c+1;
         else
            state_out = state_in; return;
         end
      end
      
   end

   function [state_out, dg, c] = reduced_BackupMotion(state_in)
      state_out = state_in;
      c = 0; dg = 0;
      for i=1:27
         r_dTheta = -params.d_theta_reverse;
         l_dTheta = -params.d_theta_reverse;
         
         R = params.r_radius*r_dTheta;    % distance Right wheel traveled
         L = params.l_radius*l_dTheta;    % distance Left wheel traveled
         
         state_out(1) = state_in(1) + (R+L)/2*cos(state_in(3));
         state_out(2) = state_in(2) + (R+L)/2*sin(state_in(3));
         state_out(3) = state_in(3);
         
         if(checkNoCollision(state_out))
            state_in = state_out; dg= dg+0.10; c = c+1;
         else
            state_out = state_in; return;
         end
      end
   end

   function [safe] = checkNoCollision(state_out)
      %*****************************
      % Transform initial car polygon into referse frame of the car
      %*****************************
      H = [cos(state_out(3)) -sin(state_out(3)) state_out(1);
           sin(state_out(3))  cos(state_out(3)) state_out(2);
           1                  1                 1          ;];
      border = H*params.border;
      
      %*****************************
      % detect collisions
      %*****************************      
      % check to see whether any of these locations falls inside our car
      % polygon
      in = inpolygon(x, y, border(1,:), border(2,:));
      
      if (sum(in)>0)
         safe = false;
      else
         safe = true;
      end
   end

end


% function [s] = createNode(state, g, parent_idx, parent_u)
% s.state = state;
% s.g   = g;
% s.pi  = parent_idx;
% s.u   = parent_u;
% end
