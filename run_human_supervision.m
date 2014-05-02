%% -----------------------------------------------------------------------

clearvars;clc;

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


%--------------------------------
% Human Training Data
%--------------------------------
t = 1;
human_training.states = [];
human_training.human_action = [];

%--------------------------------
% Display Parameters
%--------------------------------
scale = 10;
% determines the size of the map and creates a meshgrid for display purposes
[N,M] = size(map_struct.seed_map);
[x,y] = meshgrid(1:N, 1:M);
DISPLAY_ON = 1; % 1 - turns display on, 0 - turns display off
DISPLAY_TYPE = 1; % 0 - displays map as dots, 1 - displays map as blocks


%--------------------------------
% Train Existing Model
%--------------------------------
model = train_model();


% Loop through each map sample
for i = 1:length(map_struct.map_samples) 

    % Initialize the starting car state and observed map
    initialize_state;
    
    % Get path 
    %load path_1a.mat;
    load path_1b.mat;
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
        my_current_state = get_current_state(state,observed_map,path,current_path_idx,distance_to_border,square_size,M,N);
        
        
        %--------------------------------
        % Get action from model
        %--------------------------------
        model_action = my_current_state*model;    
        if(model_action < -1.5)
            model_action = -2;
        elseif (model_action < -1)
            model_action = -1;
        elseif (model_action > 1)
            model_action = 1;
        end
        
        %--------------------------------
        % Get Human Action for current state
        %--------------------------------
        action_provided = getappdata(0, 'control_selected');
        
        while(action_provided == 0)   
            pause('on');
            pause;
            action_provided = getappdata(0, 'control_selected');
        end
        human_action = getappdata(0, 'control');
        setappdata(0, 'control_selected', 0);
        
        %disp(observed_surrounding);
        disp('------------------------------');
        
        %--------------------------------
        % Store State + Human Action
        %--------------------------------
        human_training.states(t,:) = my_current_state;
        human_training.human_action(t,1) = human_action;
        t = t + 1;
        
        
        %--------------------------------
        % Execute Action from model 
        %--------------------------------
        action = model_action;
        [state, observed_map, flags] = motionModel(params, state, action, observed_map, map_struct.map_samples{i}, goal);

        if (DISPLAY_ON)
            display_environment;
        end

        
        %--------------------------------
        % update current path segment if necessary
        %--------------------------------
        if (current_path_idx < size(path.x,1))
            distance_to_end = norm( [state.x state.y] - [path.x(current_path_idx+1) path.y(current_path_idx+1)]  );
        else
            distance_to_end = norm( [state.x state.y] - [path.x(end) path.y(end)]  );
        end
        if (distance_to_end < 2.5 && current_path_idx < size(path.x,1))
            current_path_idx = current_path_idx + 1;
        end
        
        
        
        %Break if user required
        stop_simulation = getappdata(0, 'stop_simulation');
        if (stop_simulation == 1)
            break;
        end
        
    end
    
    %save('human_training/human_supervision_XXX.mat','human_training');
    
    %Break if user required
    if (stop_simulation == 1)
        break;
    end
    
end

