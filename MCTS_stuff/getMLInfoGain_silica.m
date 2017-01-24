function [ infoGain_ave, BeliefMaps ] = getMLInfoGain_silica(robot_pos, robot_orientation, BeliefMaps, sensor, DomainKnowledge, MapParameters, robot)
%calculates the expected information gained from an action through sampling

%extract the cells we expect to see from the action
[visible_cells, ~] = getVisibleCells_silica(robot_pos, robot_orientation, sensor.FOV.coords, MapParameters);

infoGain_ave = 0;

if isempty(visible_cells)
    return
end

visible_cells(1) = round(visible_cells(1)/20);
visible_cells(2) = round(visible_cells(2)/20);

if visible_cells(1) == 0
    visible_cells(1) = 1;
end

if visible_cells(2) == 0
    visible_cells(2) = 1;
end


obs_vect = zeros(1, 6);
p_sdistribution = BeliefMaps.Silica{visible_cells(1), visible_cells(2)};
[~,ml_obs] = max(p_sdistribution);

%sample from probability distribution
obs_vect(1) = ml_obs;
obs_vect(4) = 1;
obs_vect(5) = visible_cells(1);
obs_vect(6) = visible_cells(2);
[BeliefMaps, ~, infoGain_ave] = updateBeliefSpace_approx(BeliefMaps, obs_vect, robot, sensor, DomainKnowledge, MapParameters);



end










