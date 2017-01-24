%run a full iteration of the decision making cycle

%get observations
obs_vect = zeros(3,6);
obs_vect(1,:) = [3 3 3 0 70 100];
obs_vect(2,:) = [3 3 3 0 80 450];
obs_vect(3,:) = [3 3 3 0 10 10];

%update belief space
[BeliefMaps, robot] = updateBeliefSpace(BeliefMaps, obs_vect, robot, sensor, DomainKnowledge, MapParameters);

%get action space
[action_space] = getActionSpace(robot, MapParameters);


best_utility = 0;
best_action = [robot.xpos, robot.ypos, robot.orientation];

for i=1:size(action_space,1),
    robot_pos = [action_space(i,1), action_space(i,2)];
    robot_orientation = action_space(i,3);
    
    %predict observation
    [obs_predict, visible_cells] = getObsPrediction(robot_pos, robot_orientation, BeliefMaps, sensor);
    
    %calculated expected utility of the action
    [utility] = getExpectedUtility(obs_predict, BeliefMaps, sensor, DomainKnowledge, visible_cells, MapParameters);
    
    if (utility > best_utility),
        best_utility = utility;
        best_action = [action_space(i,:)];
    end
    
end

%select optimal action
opt_action = best_action;
