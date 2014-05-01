%% -----------------------------------------------------------------------

%*****************************
% Load map files and parameters
%*****************************

% load map_1.mat;
load map_2.mat;
%load map_3.mat;

load_sim_params;

% scale is used to blow up the environment for display purposes only. Set
% to whatever looks good on your screen
scale = 10;

% determines the size of the map and creates a meshgrid for display
% purposes
[N,M] = size(map_struct.seed_map);
[x,y] = meshgrid(1:N, 1:M);

DISPLAY_ON = 1; % 1 - turns display on, 0 - turns display off
DISPLAY_TYPE = 1; % 0 - displays map as dots, 1 - displays map as blocks

%% -----------------------------------------------------------------------

%*****************************
% Training/Learning Phase
%*****************************
%
% Here is where you perform any training or learning that your algorithm
% will require, such as solving an MDP, determining a policy or a value
% function, etc.
%
% NOTE: At this stage, you may access any variables provided in params or 
% map_struct EXCEPT:
%
%           map_struct.map_samples 
%
% This is your test data, don't touch them! You may however create your own 
% map_samples if you feel that this will be beneficial. You may use any of
% the functions provided (such as motion model) or create your own. Also
% note that this must be kept constant for any and all maps. You cannot
% hand tweak parameters or settings in your solver for specific maps.
%
% This code block is allowed to run as long as you'd like, provided that it
% finishes in time for you to submit the assignment!
%




% Example:
%
% myPolicy_struct = solve_mdp(params, map_struct, ...)
% ...




%% -----------------------------------------------------------------------

%*****************************
% Run Sim
%*****************************
%
% Here is where you'll actually run the simulation over the set of sampled
% maps. At each iteration, you will decide which action to take based on
% the state of the car, the observed map, and any training/learning data 
% that you may have created. There is no time limit on run time however you 
% must be able to run all map samples in time to submit the assignment.
%


% Loop through each map sample
for i = 1:length(map_struct.map_samples) 

    % Initialize the starting car state and observed map
    % observed_map is set to seed map, and the bridge information will be
    % updated once the car is within params.observation_radius
    initialize_state;

    % display the initial state
    if (DISPLAY_ON)
        display_environment;
    end

    % loop until maxCount has been reached or goal is found
    while (state.moveCount < params.max_moveCount && flags ~= 2)

        %---------------------------------------
        %
        %*****************************
        % Decide Action
        %*****************************
        %
        % Here you execute your policy (which may include re-planning or
        % any technique you consider necessary):
        
        
        % My example policy: slight turn
        action = .1;
        
        % Notice how with this policy, when the car gets close to the
        % unknown bridge (in map_1), on the first map sample the bridge 
        % becomes solid black (closed), and on the second map sample the 
        % bridge becomes white (open). Until the bridge is either 1 (white)
        % 
        % or 0 (black), you should treat it as unknown. 
        %
        % For display purposes, the unknown bridge is shown as a gray shade
        % proportional to its probability of being closed.
  
        
        %
        %---------------------------------------
        
        
        % Execute the action and update observed_map
        [state, observed_map, flags] = motionModel(params, state, action, observed_map, map_struct.map_samples{i}, goal);

        if (DISPLAY_ON)
            display_environment;
        end

        % display some output
        %[state.x state.y state.theta state.moveCount]
        
        % pause if you'd like to pause at each step
        % pause
        
    end

end
