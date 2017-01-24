function best_utility = getMLInfoGain_silica2horizon(robot_pos, robot_orientation, BeliefMaps, sensor, DomainKnowledge, MapParameters, robot)
%calculates the expected information gained from an action through sampling

%calculates the utility from the first action
[utility1, BeliefMaps] = getMLInfoGain_silica(robot_pos, robot_orientation, BeliefMaps, sensor, DomainKnowledge, MapParameters, robot);

robot_new.xpos = robot_pos(1);
robot_new.ypos = robot_pos(2);
action_space = 0;
[reachable_action_space, reachable_action_space_silica] = getActionSpace_new(robot_new, MapParameters, action_space);

cost_remote = 1;
cost_local = 8;
best_utility = 0;
%Evaluate remote sensing actions
for i=1:size(reachable_action_space,1),
    robot_pos = [reachable_action_space(i,1), reachable_action_space(i,2)];
    robot_orientation = reachable_action_space(i,3);

    %[utility, utility_std] = getExpectedInfoGain(robot_pos, robot_orientation, BeliefMaps, sensor, DomainKnowledge, MapParameters, robot);
    [utility2, ~] = getMLInfoGain(robot_pos, robot_orientation, BeliefMaps, sensor, DomainKnowledge, MapParameters, robot);
    
    utility2 = (utility1+utility2)/(cost_remote+5);
    
    if (utility2 > best_utility),
        best_utility = utility2;
    end
    
end

%Evaluate UV sensing actions
for i=1:size(reachable_action_space_silica,1),
    robot_pos = [reachable_action_space_silica(i,1), reachable_action_space_silica(i,2)];
    robot_orientation = reachable_action_space_silica(i,3);

    [utility2, ~] = getMLInfoGain_silica(robot_pos, robot_orientation, BeliefMaps, sensor, DomainKnowledge, MapParameters, robot);
    
    utility2 = (utility1+utility2)/(cost_local+5);
    
    if (utility2 > best_utility),
        best_utility = utility2;
    end
    
end

end












