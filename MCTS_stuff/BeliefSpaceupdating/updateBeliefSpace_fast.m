function [ BeliefMaps, robot, infoGain_sum ] = updateBeliefSpace_fast( BeliefMaps, obs_vect, robot, sensor, DomainKnowledge, MapParameters  )
%reads observations from CV module- transforms them wrt to the map
%updates the belief space through exact Bayesian inference
%assumes known conditional probability parameters

robot_pos = [robot.xpos robot.ypos];
robot_orientation = robot.orientation;


infoGain_sum = 0;

if obs_vect(1,4) == 1,
    %only silica measured
    remote_sense_flag = 0;
else
    %remote sensor used
    remote_sense_flag = 1;
end


%list of cells that were seen
[visible_cells, ~] = getVisibleCells(robot_pos, robot_orientation, sensor.FOV.coords );

if remote_sense_flag == 1, %remote sense made
    idx = sub2ind(size(robot.visibility), visible_cells(:,1), visible_cells(:,2));
    robot.visibility(idx) = 1;
end

%associate data with points on the map- returns observations along with
%points on the map they correspond to

%to do
%obs_vect = obs_trans(obs_vect, getVisibleCells, robot_pos, robot_orientation);

%treat observations as sequential and update
%for..

for i = 1:size(obs_vect,1)
    robot.visibility(obs_vect(i,5), obs_vect(i,6)) = 2;
    if remote_sense_flag == 1,
    %just observations of rocks taken

        BeliefMaps = update_fspace(obs_vect(i,:), BeliefMaps, sensor);
        [BeliefMaps, p_zgivenr] = update_rspace(obs_vect(i,:), DomainKnowledge, BeliefMaps, sensor);
        
        %updates belief on location where observations were taken and the
        %area around it
       % disp('New obs');
        %tic
        [BeliefMaps, infoGain] = update_lspace_approx(obs_vect(i,:), DomainKnowledge, BeliefMaps, p_zgivenr, MapParameters);
        %toc
        
       % tic
        %BeliefMaps2 = update_lspace_fast(obs_vect(i,:), DomainKnowledge, BeliefMaps, p_zgivenr, MapParameters);
       % toc
        
        %update unobserved parts of the map- based on new location
        %estimates calculate rock and feature beliefs
        BeliefMaps = updateALL(obs_vect(i,:), BeliefMaps, DomainKnowledge, MapParameters, robot);
        %BeliefMaps = update_sspace(obs_vect(i), DomainKnowledge, BeliefMaps);
        
        
        
    else
    %silica content directly measured- single observation
        BeliefMaps = update_lspace_s(obs_vect(i,:), DomainKnowledge, BeliefMaps, MapParameters);
        BeliefMaps = update_ALL_s(obs_vect, DomainKnowledge, BeliefMaps, MapParameters, robot);
        %BeliefMaps = update_sspace_s(obs_vect(i,:), DomainKnowledge, BeliefMaps, MapParameters);
        
    %update r&f space- this is difficult to update in locations where
    %observations have been taken already but straightforward elsewhere
    %read visibility map- if cell hasn't been observed use:    
        %BeliefMaps = update_rspace_s(obs_vect(i,:), DomainKnowledge, BeliefMaps, MapParameters, robot);
        %BeliefMaps = update_fspace_s(obs_vect(i,:), DomainKnowledge, BeliefMaps, MapParameters, robot);
   
    %otherwise- ignore for now as it does not make a big difference 
    %ignore rock and feature belief update
    
    end
    
    infoGain_sum = infoGain + infoGain_sum;
end



end



