function [BeliefMaps, infoGain, robot] = simulate_BeliefUpdate(BeliefMaps, sensor, DomainKnowledge, MapParameters, robot, iteration_count)
%calculates the expected information gained from an action through sampling

infoGain = 0;

robot_pos = [robot.xpos, robot.ypos];
robot_orientation = robot.orientation;

%extract the cells we expect to see from the action
[visible_cells, ~] = getVisibleCells(robot_pos, robot_orientation, sensor.FOV.coords, MapParameters);

if isempty(visible_cells)
    return
end

obs_vect = zeros(length(visible_cells), 6);
for i=1:length(visible_cells),
    rand_sample = rand;
    
    %check which of the cells have been observed already
    if robot.visibility(visible_cells(i,1),visible_cells(i,2)) == 1,
        %rock wasnt there before so we shouldnt see it again
        continue;
        
    elseif ((robot.visibility(visible_cells(i,1),visible_cells(i,2)) == 2) || (rand_sample < 0.0015)), %there is a rock already there or its there in the sample
        %if there was a rock we expect to see a rock again
        f_prob1 = BeliefMaps.F1{visible_cells(i,1), visible_cells(i,2)};
        f_prob2 = BeliefMaps.F2{visible_cells(i,1), visible_cells(i,2)};
        f_prob3 = BeliefMaps.F3{visible_cells(i,1), visible_cells(i,2)};
        
        p_z1 = sensor.noise_f1*f_prob1';
        p_z2 = sensor.noise_f1*f_prob2';
        p_z3 = sensor.noise_f1*f_prob3';
        
        %sample from probability distribution
        sample_obs = mnrnd(1, p_z1);
        obs_vect(i,1) = find(sample_obs == 1);
        
        sample_obs = mnrnd(1, p_z2);
        obs_vect(i,2) = find(sample_obs == 1);
        
        sample_obs = mnrnd(1, p_z3);
        obs_vect(i,3) = find(sample_obs == 1);
        obs_vect(i,4) = 0;
        obs_vect(i,5) = visible_cells(i,1);
        obs_vect(i,6) = visible_cells(i,2);
    else
        continue;
    end
end

obs_vect(obs_vect(:,1) == 0, :) = [];

if isempty(obs_vect),
    return    
elseif iteration_count < 5,
    [BeliefMaps, robot, infoGain] = updateBeliefSpace(BeliefMaps, obs_vect, robot, sensor, DomainKnowledge, MapParameters);
else
    [BeliefMaps, robot, infoGain] = updateBeliefSpace_approx(BeliefMaps, obs_vect, robot, sensor, DomainKnowledge, MapParameters);
end


end








