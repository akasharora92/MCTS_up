function [ action_sequence_tot ] = create_path( robot_xpos, robot_ypos )
%create a uniform exhaustive sampling path based on start location

%we want the robot to take both remote sense and local sense actions
num_actions = 50;

%initialise sequence- x,y,orientation,sensing mode
action_sequence_tot = [];
action_mult = 20;

%direction labels
%0 is stay
%1 is up
%2 is left
%3 is right
%4 is down

%direction_sequence = ['stay', 'up', 'left', 'down', 'down', 'right', 'right', 'up', 'up', 'up', 'left', 'left', 'left', 'down'];
direction_sequence = [0,1,1,1,1,1,1,1,1,1];

new_robot_pos = [robot_xpos, robot_ypos];

for i = 1:length(direction_sequence),
    %figure out the next action based on the iteration
    if direction_sequence(i) == 0, %stay
        new_robot_pos = new_robot_pos;
    elseif direction_sequence(i) == 1, %up
        new_robot_pos = new_robot_pos + [0 20];
    elseif direction_sequence(i) == 2, %left
        new_robot_pos = new_robot_pos + [-20 0];
    elseif direction_sequence(i) == 3, %right
        new_robot_pos = new_robot_pos + [20 0];
    elseif direction_sequence(i) == 4, %down
        new_robot_pos = new_robot_pos + [0 -20];
    end
    
    action_sequence = zeros(6,4);
    action_sequence(:,1) = new_robot_pos(1)*ones(6,1);
    action_sequence(:,2) = new_robot_pos(2)*ones(6,1);
    action_sequence(:,3) = [0;0;90;90;270;270];
    action_sequence(:,4) = [0;1;0;1;0;1];
    action_sequence_tot = [action_sequence_tot;action_sequence];
    
end

action_sequence_tot = action_sequence_tot(1:num_actions,:);

end

