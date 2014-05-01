%% -----------------------------------------------------------------------

%--------------------------------
% Load map files and parameters
%--------------------------------

load all_maps/map_1.mat;
%load all_maps/map_2.mat;
%load all_maps/map_3.mat;
%load all_maps/map_4.mat;
%load all_maps/map_5.mat;
%load all_maps/map_6.mat;
%load all_maps/map_7.mat;

load_sim_params;




%--------------------------------
% Human Training Parameters
%--------------------------------
stop_simulation = 0;
setappdata(0, 'stop_simulation', 0);
setappdata(0, 'control', 0);
setappdata(0, 'control_selected', 0);

%--------------------------------
% Feature State
%--------------------------------
% A: square around car
% B: car orientation
% C: angle between orientation and path
%--------------------------------
distance_to_border = 4;
square_size = 1 + 2*distance_to_border;
feature_size = square_size*square_size + 1 + 1;

%--------------------------------
% Human Training Data
%--------------------------------
t = 1;
states = zeros(1,feature_size);
human_action = zeros(1);

%--------------------------------
% Display Parameters
%--------------------------------
scale = 10;
% determines the size of the map and creates a meshgrid for display purposes
[N,M] = size(map_struct.seed_map);
[x,y] = meshgrid(1:N, 1:M);
DISPLAY_ON = 1; % 1 - turns display on, 0 - turns display off
DISPLAY_TYPE = 1; % 0 - displays map as dots, 1 - displays map as blocks


% Loop through each map sample
for i = 1:length(map_struct.map_samples) 

    % Initialize the starting car state and observed map
    initialize_state;
    
    % Get path 
    load path_1a.mat;
    current_path_idx = 1;
    

    % display the initial state
    if (DISPLAY_ON)
        display_environment;
        set(fig,'KeyPressFcn',@keys_listener);
    end
    
    %collect points for path
    %[x_path,y_path] = ginput;
    

    % loop until maxCount has been reached or goal is found
    while (state.moveCount < params.max_moveCount && flags ~= 2)
        
        %--------------------------------
        % Get Current Features
        %--------------------------------
        %square around car
        current_state = zeros(1,feature_size);
        x_cur = round(state.x); 
        y_cur = round(state.y);
        observed_surrounding = zeros(square_size,square_size);
        x_min = max(1,x_cur - distance_to_border);
        x_max = min(M,x_cur + distance_to_border);
        y_min = max(1,y_cur - distance_to_border);
        y_max = min(N,y_cur + distance_to_border);
        center = distance_to_border + 1;
        pos_y_min = center-(y_cur-y_min);
        pos_y_max = center+(y_max-y_cur);
        pos_x_min = center-(x_cur-x_min);
        pos_x_max = center+(x_max-x_cur);
        observed_surrounding(pos_y_min:pos_y_max,pos_x_min:pos_x_max) = observed_map(y_min:y_max,x_min:x_max);
        %orientation
        observed_theta = mod(state.theta,2*pi);
        if(observed_theta < 0)
            observed_theta = observed_theta + 2*pi;
        end
        %angle between orientation and path
        current_path_vector = [path.x(current_path_idx+1) path.y(current_path_idx+1)]-[path.x(current_path_idx) path.y(current_path_idx)];
        reference_vector = [1 0];
        path_orientation = acos( dot(current_path_vector, reference_vector) / (norm(current_path_vector)*norm(reference_vector)) );
        angle_difference = path_orientation - observed_theta;
        
        
        %--------------------------------
        % Get Human Action for current state
        %--------------------------------
        action_provided = getappdata(0, 'control_selected');
        
        while(action_provided == 0)   
            pause('on');
            pause;
            action_provided = getappdata(0, 'control_selected');
        end
        action = getappdata(0, 'control');
        setappdata(0, 'control_selected', 0);
        
        %disp(observed_surrounding);
        disp('------------------------------');
        
        %--------------------------------
        % Store State + Human Action
        %--------------------------------
        row_observed_surrounding = reshape(observed_surrounding',1,numel(observed_surrounding));
        states(t,:) = [row_observed_surrounding,observed_theta,angle_difference];
        human_action(t,1) = action;
        t = t + 1;
        
        %--------------------------------
        % Execute Action and Update observed_map
        %--------------------------------
        [state, observed_map, flags] = motionModel(params, state, action, observed_map, map_struct.map_samples{i}, goal);

        if (DISPLAY_ON)
            display_environment;
        end

        
        %--------------------------------
        % update current path segment if necessary
        %--------------------------------
        distance_to_end = norm( [state.x state.y] - [path.x(current_path_idx+1) path.y(current_path_idx+1)]  );
        if (distance_to_end < 2.5 && current_path_idx < size(path.x,1))
            current_path_idx = current_path_idx + 1;
        end
        
        
        
        %Break if user required
        stop_simulation = getappdata(0, 'stop_simulation');
        if (stop_simulation == 1)
            break;
        end
        
    end
    
    %Break if user required
    if (stop_simulation == 1)
        break;
    end
    
end

