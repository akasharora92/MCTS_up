robot_pos = [20 10];
robot_orientation = 45;


%list of cells that were seen
[visible_cells, new_FOV ] = getVisibleCells(robot_pos, robot_orientation, sensor.FOV.coords );
idx = sub2ind(size(robot.visibility), visible_cells(:,1), visible_cells(:,2));
robot.visibility(idx) = 1;
%observations that were taken- from CV module
obs_vect = [];

if obs_vect(1,4) == 1,
    %only rocks measured
    remote_sense_flag = 0;
else
    %silica measured
    remote_sense_flag = 1;
end

%associate data with points on the map- returns observations along with
%points on the map they correspond to
obs_vect = obs_trans(obs_vect, getVisibleCells, robot_pos, robot_orientation);

%treat observations as sequential and update
%for..

for i = 1:length(obs_vect)
    if remote_sense_flag == 1,
    %just observations of rocks taken
        BeliefMaps = update_fspace(obs_vect(i), DomainKnowledge, BeliefMaps);
        [BeliefMaps, p_zgivenr] = update_rspace(obs_vect(i), DomainKnowledge, BeliefMaps);
        
        %updates belief on location where observations were taken and the
        %area around it
        BeliefMaps = update_lspace(obs_vect(i), DomainKnowledge, BeliefMaps, p_zgivenr, MapParameters);
        BeliefMaps = update_sspace(obs_vect(i), DomainKnowledge, BeliefMaps);
        %update unobserved parts of the map- based on new location
        %estimates calculate rock and feature beliefs
        
        
    else
    %silica content directly measured- single observation
        BeliefMaps = update_lspace_s(obs_vect(i,:), DomainKnowledge, BeliefMaps, MapParameters);
        BeliefMaps = update_sspace_s(obs_vect(i,:), DomainKnowledge, BeliefMaps, MapParameters);
        
    %update r&f space- this is difficult to update in locations where
    %observations have been taken already but straightforward elsewhere
    %read visibility map- if cell hasn't been observed use:    
        BeliefMaps = update_rspace_s(obs_vect(i,:), DomainKnowledge, BeliefMaps, MapParameters, robot);
        BeliefMaps = update_fspace_s(obs_vect(i,:), DomainKnowledge, BeliefMaps, MapParameters, robot);
   
    %otherwise- ignore for now as it does not make a big difference 
    %ignore rock and feature belief update
    
    end
end

