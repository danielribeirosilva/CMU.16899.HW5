function [] = LPA(s_start, s_goal, map)
% Input
%    - s_start
%    - s_goal
%    - map
%    

% Build the heuristic map
h_map = floodfill(map, s_goal);

max_cells = 1000;
node_list = cell(max_cells);
num_cells = 1;

pq = pq_create(max_cells);

h = interp2(h_map, s_start.y, s_start.x, 'nearest');
node_list(num_cells) = create_state(s_start, 0, 0, 0);
pq_push(pq, num_cells, h);

goal_reached = false;
% while 
% 1. the list is not empty, 
% 2. the goal has not reached
% 3. the total cell expaned has not exceeded max_cells
while(pq_size(pq)>0 && num_cells <= max_cells)
   % Take the current best node
   [idx, cost] = pq_pop(pq);
   s_node = node_list(idx);
   
   % Check if the goal is reached
   if ( isclose(s_node.state, s_goal))
      goal_reached = true;
      break;
   end
   
   % Expand this node: [0 1 -1 -2]
   s_out = reduced_ForwardMotion(s_node.state);
   if (checkNoCollision(s_out))
      num_cells = num_cells+1;
      new_node = createNode(s_out, s_node.state.g+0.15, idx, 0);
      node_list{num_cells} = new_node;
      h = interp2(h_map, new_node.state.y, new_state.state.x, 'nearest');
      pq_push(pq, num_cells, new_state.g+h);
      max_steps = max([max_steps, s_out.moveCount]);
   end
   
   s_out = reduced_LeftMotion(s_node.state);
   if (checkNoCollision(s_out))
      num_cells = num_cells+1;
      new_node = createNode(s_out, s_node.state.g+0.15, idx, 1);
      node_list{num_cells} = new_node;
      h = interp2(h_map, new_node.state.y, new_state.state.x, 'nearest');
      pq_push(pq, num_cells, new_state.g+h);
      max_steps = max([max_steps, s_out.moveCount]);
   end
   
   s_out = reduced_RightMotion(s_node.state);
   if (checkNoCollision(s_out))
      num_cells = num_cells+1;
      new_node = createNode(s_out, s_node.state.g+0.15, idx, -1);
      node_list{num_cells} = new_node;
      h = interp2(h_map, new_node.state.y, new_state.state.x, 'nearest');
      pq_push(pq, num_cells, new_state.g+h);
      max_steps = max([max_steps, s_out.moveCount]);
   end
   
   s_out = reduced_RightMotion(s_node.state);
   if (checkNoCollision(s_out))
      num_cells = num_cells+1;
      new_node = createNode(s_out, s_node.state.g+0.15, idx, -2);
      node_list{num_cells} = new_node;
      h = interp2(h_map, new_node.state.y, new_state.state.x, 'nearest');
      pq_push(pq, num_cells, new_state.g+h);
      max_steps = max([max_steps, s_out.moveCount]);
   end
   
   % Debug show expansion:
   if
   line(scale*[state.border(1,:); state.border(1,[2:end 1])], scale*[state.border(2,:); state.border(2,[2:end 1])], 'Color','Red', 'LineSmoothing', 'on');
end

% If goal is reached, recover the path, else return error message
if (goal_reached)
   % return path
else
   % return error
end

end

function [s] = createNode(state, g, parent_idx, parent_u)
s.state = state;
s.g   = g;
s.pi  = parent_idx;
s.u   = parent_u;
end

function [safe] = checkNoCollision(state_out)
%*****************************
% Transform initial car polygon into referse frame of the car
%*****************************
global params;
global plan_map;
state_out.H = [cos(state_out.theta) -sin(state_out.theta) state_out.x;
               sin(state_out.theta)  cos(state_out.theta) state_out.y;
               1                     1                    1          ;];
state_out.border = state_out.H*params.border;


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
in = inpolygon(x(ind), y(ind), state_out.border(1,:), state_out.border(2,:));

if (sum(in)>0) 
   safe = false;
else
   safe = true;
end
end

function [state_out] = reduced_leftMotion(state_in)
global params;

r_dTheta = params.d_theta_nom + params.d_theta_max_dev;
l_dTheta = params.d_theta_nom - params.d_theta_max_dev;
    
R = params.r_radius*r_dTheta;    % distance Right wheel traveled
L = params.l_radius*l_dTheta;    % distance Left wheel traveled

state_out.x = state_in.x + params.wb/2*(R+L)/(R-L)*(sin((R-L)/params.wb + state_in.theta) - sin(state_in.theta));
state_out.y = state_in.y - params.wb/2*(R+L)/(R-L)*(cos((R-L)/params.wb + state_in.theta) - cos(state_in.theta));
state_out.theta = state_in.theta + (R-L)/params.wb;
state_out.moveCount = state_in.moveCount + 1;
end

function [state_out] = reduced_RightMotion(state_in)
global params;

r_dTheta = params.d_theta_nom - params.d_theta_max_dev;
l_dTheta = params.d_theta_nom + params.d_theta_max_dev;
    
R = params.r_radius*r_dTheta;    % distance Right wheel traveled
L = params.l_radius*l_dTheta;    % distance Left wheel traveled

state_out.x = state_in.x + params.wb/2*(R+L)/(R-L)*(sin((R-L)/params.wb + state_in.theta) - sin(state_in.theta));
state_out.y = state_in.y - params.wb/2*(R+L)/(R-L)*(cos((R-L)/params.wb + state_in.theta) - cos(state_in.theta));
state_out.theta = state_in.theta + (R-L)/params.wb;
state_out.moveCount = state_in.moveCount + 1;
end

function [state_out] = reduced_ForwardMotion(state_in)
global params;

r_dTheta = params.d_theta_nom;
l_dTheta = params.d_theta_nom;
    
R = params.r_radius*r_dTheta;    % distance Right wheel traveled
L = params.l_radius*l_dTheta;    % distance Left wheel traveled

state_out.x = state_in.x + (R+L)/2*cos(state_in.theta);
state_out.y = state_in.y + (R+L)/2*sin(state_in.theta);
state_out.theta = state_in.theta + (R-L)/params.wb;
state_out.moveCount = state_in.moveCount + 1;
end

function [state_out] = reduced_BackupMotion(state_in)
global params;
    
r_dTheta = -params.d_theta_reverse;
l_dTheta = -params.d_theta_reverse;
    
R = params.r_radius*r_dTheta;    % distance Right wheel traveled
L = params.l_radius*l_dTheta;    % distance Left wheel traveled
        
% Car moved straight backwards
state_out.x = state_in.x + (R+L)/2*cos(state_in.theta);
state_out.y = state_in.y + (R+L)/2*sin(state_in.theta);
   
state_out.theta = state_in.theta + (R-L)/params.wb;
state_out.moveCount = state_in.moveCount + 1;
end