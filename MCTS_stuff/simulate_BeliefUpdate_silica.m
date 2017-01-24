function [BeliefMaps, infoGain, robot] = simulate_BeliefUpdate_silica(BeliefMaps, sensor, DomainKnowledge, MapParameters, robot, iteration_count)
%Samples a silica observation based on belief maps and outputs the info
%gain

infoGain = 0;

robot_pos = [robot.xpos, robot.ypos];
robot_orientation = robot.orientation;

%extract the cells we expect to see from the action
[visible_cells, ~] = getVisibleCells_silica(robot_pos, robot_orientation, sensor.FOV.coords, MapParameters);

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

%generate random observation
obs_vect = zeros(1, 6);
p_sdistribution = BeliefMaps.Silica{visible_cells(1), visible_cells(2)};

%sample from probability distribution
sample_obs = mnrnd(1, p_sdistribution);
obs_vect(1) = find(sample_obs == 1);

obs_vect(4) = 1;
obs_vect(5) = visible_cells(1);
obs_vect(6) = visible_cells(2);

if iteration_count < 5
    [BeliefMaps, robot, infoGain] = updateBeliefSpace(BeliefMaps, obs_vect, robot, sensor, DomainKnowledge, MapParameters);
else
    [BeliefMaps, robot, infoGain] = updateBeliefSpace_approx(BeliefMaps, obs_vect, robot, sensor, DomainKnowledge, MapParameters);
end


end

