function [num_node] = rrt(s_start, s_goal, map)
% Inputs
%
%
max_node = 10000;
p_list = zeros(max_node, 1);
u_list = zeros(max_node, 1);
s_list = zeros(max_node, 3);

goal_bias = 0.05;
rng(1173);

[ri, ci] = find(map==1);
numOpenCell = numel(ri);

% Add in the first node:
num_node = 1;
s_list(num_node,:) = s_start;
u_list(num_node) = NaN;
p_list(num_node) = 0;


while(num_node < max_node)
   % We pick the goal directly for some percentage
   if rand(1) < goal_bias
      q_rand = s_goal(1:2);
   else
      idx = randi([1, numOpenCell], 1);
      q_rand = [ri(idx), ci(idx)];
   end
   
   % Find the nearest node in the tree
   [idx, ~] = knnsearch(s_list(1:num_node,[1,2]), q_rand, 'K', k);
   
   % Steer the q_near towards q_rand
   [q_new, u, flag] = steer(s_list(idx(1), :), q_rand);
   
   % If such steer is successful, add it to the tree
   if(flag)
      num_node = num_node+1;
      s_list(num_node, :) = q_new;
      u_list(num_node) = u;
      p_list(num_node) = idx(1);
      
      
   if (num_node > 1)
      scale = 10;
      p_state = s_list(p_list(num_node),:);
      line(scale*[p_state(1), q_new(1)], scale*[p_state(2), q_new(2)], 'Color','Red', 'LineSmoothing', 'on');
      if(mod(num_node, 100)==0)
         pause(0.05);
      end
   end
   
   end
end

end

function [q_new, u, flag] = steer(n_state, r_state)
global params;

v1 = [cos(n_state(3)), sin(n_state(3))];
v2 = [r_state(1)-n_state(1), r_state(2)-n_state(2)];
nv2 = norm(v2);

% If r_s is too close to n_s, then quit
if (nv2 < 1e-3)
   flag = false;
   q_new = [0 0 0];
   u = 0;
   return;
end
v2 = v2/nv2;

angle = atan2(v2(2), v2(1)) - atan2(v1(2), v1(1));
if (angle > pi)
   angle = angle - 2*pi;
elseif(angle < -pi)
   angle = angle + 2*pi;
end

if (angle > pi/2 || angle < -pi/2)
   u = -2;
else
   u = angle/params.d_theta_max_dev;
   if (u > 1.0)
      u = 1.0;
   elseif (u < -1.0)
      u = -1.0;
   end
end   

q_new = robot_motion(n_state, u);

flag = checkNoCollision(q_new);
end

function [state_out] = robot_motion(state_in, action)

state_out = [0 0 0];
precision = 1e-4;
global params;

if (action == -2)    % handle reverse motion
   r_dTheta = -params.d_theta_reverse;
   l_dTheta = -params.d_theta_reverse;
   
   R = params.r_radius*r_dTheta;    % distance Right wheel traveled
   L = params.l_radius*l_dTheta;    % distance Left wheel traveled
   
   % Car moved straight backwards
   state_out(1) = state_in(1) + (R+L)/2*cos(state_in(3) );
   state_out(2) = state_in(2) + (R+L)/2*sin(state_in(3) );
else
   r_dTheta = params.d_theta_nom + params.d_theta_max_dev*action;
   l_dTheta = params.d_theta_nom - params.d_theta_max_dev*action;
    
   R = params.r_radius*r_dTheta;    % distance Right wheel traveled
   L = params.l_radius*l_dTheta;    % distance Left wheel traveled

   if (norm(R-L)<precision)
      % Car moved straight
      state_out(1) = state_in(1) + (R+L)/2*cos(state_in(3) );
      state_out(2) = state_in(2) + (R+L)/2*sin(state_in(3) );
   else
      % Car moved along an arc
      state_out(1) = state_in(1) + params.wb/2*(R+L)/(R-L)*(sin((R-L)/params.wb + state_in(3)) - sin(state_in(3)));
      state_out(2) = state_in(2) - params.wb/2*(R+L)/(R-L)*(cos((R-L)/params.wb + state_in(3)) - cos(state_in(3)));
   end
end

angle = state_in(3) + (R-L)/params.wb;
if angle > pi
   state_out(3) = angle - 2*pi;
elseif angle < -pi
   state_out(3) = angle + 2*pi;
else
   state_out(3) = angle;
end
end

function [safe] = checkNoCollision(state_out)
%*****************************
% Transform initial car polygon into referse frame of the car
%*****************************
global params;
global plan_map;

H = [cos(state_out(3)) -sin(state_out(3)) state_out(1);
     sin(state_out(3))  cos(state_out(3)) state_out(2);
     1                  1                 1          ;];
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



