%This function creates a sequence of actions using a random policy until
%the budget is exhausted

%INPUTS: Current robot position, its children, budget
%OUTPUTS: Sequence of states and actions taken while adherring to budget

function [state_sequence] = rollout_randompolicy(current_node, budget, MapParameters, state_sequence_init)

robot.xpos = current_node.x_pos;
robot.ypos = current_node.y_pos;
robot.orientation = current_node.robot_or;

% pick random actions until budget is exhausted
sequence = current_node.sequence;

state_sequence = state_sequence_init;

while cost(sequence) < budget,
    [unpicked_children] = getChildren(robot, MapParameters);
    index = randi(size(unpicked_children,1));
    new_child = [unpicked_children(index,:)];
    
    % check if using local sensor will break the budget
    if (cost(sequence) + cost(unpicked_children(index,4)) > budget), 
        %replace with remote sensor if so
        sequence = [sequence, 0];
        new_child = [unpicked_children(index,1:3), 0];
        state_sequence = [state_sequence; new_child];       
    else
        sequence = [sequence, unpicked_children(index,4)];
        state_sequence = [state_sequence; new_child];
    end
    
    robot.xpos = new_child(1);
    robot.ypos = new_child(2);
    robot.orientation = new_child(3);
    
end

end

