function [ action_sequence ] = create_path( robot_xpos, robot_ypos, budget )
%this function takes a start position for a robot and plans a fixed path
%the path involves the robot looking straight, looking left, looking right,
%local sensor straight, then move one step forward

remaining_budget = budget;

action_sequence = [];

%sensing order: look left, look forward, look right, local forward.

state = 1;

current_xpos = robot_xpos;
current_ypos = robot_ypos;

while remaining_budget > 0,
    if state == 1, %look left
        next_action = [current_xpos, current_ypos, -90, 0];
        remaining_budget = remaining_budget - 1;
    elseif state == 2, %look forward
        next_action = [current_xpos, current_ypos, 0, 0];
        remaining_budget = remaining_budget - 1;
    elseif state == 3, %look right
        next_action = [current_xpos, current_ypos, 90, 0];
        remaining_budget = remaining_budget - 1;
    elseif state == 4, %local sensor forward
        if remaining_budget > 8, %check if robot has budget to use the local sensor
            next_action = [current_xpos, current_ypos, 0, 1];
            remaining_budget = remaining_budget - 8;
        else
            next_action = [current_xpos, current_ypos, 0, 0];
            remaining_budget = remaining_budget - 1;
        end
    else %move forward
        current_ypos = current_ypos + 1;
        next_action = [current_xpos, current_ypos, 0, 0];
        remaining_budget = remaining_budget - 1;
    end
    
    action_sequence = [action_sequence; next_action];
    if state == 5,
        state = 1;
    else
        state = state + 1;
    end
end

end

