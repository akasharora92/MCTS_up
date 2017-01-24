function [utility] = getExpectedUtility(obs_predict, BeliefMaps, sensor, DomainKnowledge, visible_cells, MapParameters)
%calculates the expected utility of the action based on the predicted
%observations

%EU(a) = int P(Z|a).U(Z).dZ
%P(Z|a) = probability of observations for each visible cell- given
%if we sum over all possible observations it will take a lot of time as
%there are 3^N different sets of observations that can be made
%we can use maximum likelihood obs for each cell- we can then weigh the
%potential info gain based on the entropy

%we can also find the expected observations- i.e. number of each feature we
%are likely to observe. In a FOV of 50 cells- we predict there will 4
%circular rocks, 3 high texture rocks, etc

%first we need a good utility function
%needs to capture the goal, be fast to evaluate and be mathematically sound
%information gain is popular. we need to calculate H(S|Z1, Z2, Z3)

%for Z1 = i
%for Z2 = j
%for Z3 = k 
%iterate
%P(S|Z1, Z2, Z3) --> H(S|Z1, Z2, Z3)
%sum over all possible combinations of Z1, Z2, Z3 

%get coordinates of which cell each obs belongs to- use visible cells

BeliefMaps_try = BeliefMaps;


% %number of observations we expect to make
% obs_num = size(obs_predict,1);
% 
% %creating all combinations of observations
% obs_vect = [1 2 3]; %three possible values for each obs
% sets = {};
% for i=1:obs_num,
%     sets{end+1} = [1 2 3];
% end
% 
% [x, y, z] = ndgrid(sets{:});
% obs_combo = [x(:) y(:) z(:)];
% 
% for i=1:size(obs_combo,1),
%     BeliefMaps_try = BeliefMaps;   
% end

%assume in each cell we can only observe 1, 2 or 3- just one feature
%use maximum likelihood assumption
obs_num = size(obs_predict,1);

for i=1:obs_num,
    [~, ml_obs_1] = max(obs_predict(i,1:3));
    [~, ml_obs_2] = max(obs_predict(i,4:6));
    [~, ml_obs_3] = max(obs_predict(i,7:9));
    obs_vect = [ml_obs_1, ml_obs_2, ml_obs_3 0 visible_cells(i,1), visible_cells(i,2)]; %ignoring local sensor for now
    
    BeliefMaps_try = update_fspace(obs_vect, BeliefMaps_try, sensor);
    [BeliefMaps_try, p_zgivenr] = update_rspace(obs_vect, DomainKnowledge, BeliefMaps_try, sensor);
    BeliefMaps_try = update_lspace(obs_vect, DomainKnowledge, BeliefMaps_try, p_zgivenr, MapParameters);
    %BeliefMaps_try = updateALL(obs_vect(i,:), BeliefMaps_try, DomainKnowledge, MapParameters, robot);
    BeliefMaps_try = update_sspace(obs_vect, DomainKnowledge, BeliefMaps_try, MapParameters);
disp(i)
end

%calculate change in entropy for the silica field
utility = 0;
% init_Silica_map = BeliefMaps.Silica;
% for i=1:size(init_Silica_map)

end

