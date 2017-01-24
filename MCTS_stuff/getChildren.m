function [ unpicked_children ] = getChildren(robot, MapParameters)
%get list of children

[reachable_action_space, reachable_action_space_silica] = getActionSpace_new(robot, MapParameters);

if isempty(reachable_action_space),
    unpicked_children = [];
else
    unpicked_children = [reachable_action_space; reachable_action_space_silica];
    sense_vector = [zeros(size(reachable_action_space,1),1); ones(size(reachable_action_space_silica,1),1)];
    unpicked_children = [unpicked_children, sense_vector];
end


end

