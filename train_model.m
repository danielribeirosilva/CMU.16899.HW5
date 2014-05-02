function [model] = train_model()
    
    %get all training data
    training_data = put_data_together();
    
    %linear regression data
    model = regress(training_data.human_action,training_data.states);

    
    
    
%----------------------------------------------------------------
% AUXILIARY FUNCTIONS
%----------------------------------------------------------------


%--------------------------------
% merge different training data
%--------------------------------
function [training_data] = put_data_together()
    folder_name = 'human_training/';
    file_names = {%'human_training_1.mat';%bad
                  'human_training_2.mat';%ok
                  'human_training_3.mat';%ok
                  %'human_training_4.mat';%ok-
                  %'human_training_5.mat';%ok-
                  %'human_training_6.mat';%bad
                  'human_training_7.mat';
                  'human_supervision_1.mat';
                  'human_supervision_2.mat';
                  'human_supervision_3.mat';
                  %'human_supervision_4.mat';
                  %'human_supervision_5.mat';
                  'human_supervision_6.mat';
                  };

    training_data.states = [];
    training_data.human_action = [];
    
    for i=1:size(file_names,1)
        file_path = strcat(folder_name,file_names(i));
        data = load(file_path{1});
        training_data.states = [training_data.states;data.human_training.states];
        training_data.human_action = [training_data.human_action;data.human_training.human_action];
    end
              
end




end
