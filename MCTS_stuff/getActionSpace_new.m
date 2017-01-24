function [reachable_action_space, reachable_action_space_silica] = getActionSpace_new(robot, MapParameters)
%get a set of possible actions based on the robot's current position,
%action space and restrictions on the map

possible_orientations = [-90, -45, 45, 90];
possible_orientations = possible_orientations + robot.orientation*ones(1,4);

%scale observation to -180 to 180
for i=1:length(possible_orientations),
   if possible_orientations(i) < -135,
       possible_orientations(i) = possible_orientations(i) + 360;
   elseif possible_orientations(i) > 180,
       possible_orientations(i) = possible_orientations(i) - 360;
   end
end

if (robot.orientation == 0)
    new_x = robot.xpos + 1;
    new_y = robot.ypos;
elseif (robot.orientation == 45)
    new_x = robot.xpos + 1;
    new_y = robot.ypos + 1;
elseif (robot.orientation == 90)
    new_x = robot.xpos;
    new_y = robot.ypos + 1;
elseif (robot.orientation == 135)
    new_x = robot.xpos - 1;
    new_y = robot.ypos + 1;
elseif (robot.orientation == 180)
    new_x = robot.xpos - 1;
    new_y = robot.ypos;
elseif (robot.orientation == -45)
    new_x = robot.xpos + 1;
    new_y = robot.ypos - 1;
elseif (robot.orientation == -90)
    new_x = robot.xpos;
    new_y = robot.ypos - 1;
elseif (robot.orientation == -135)
    new_x = robot.xpos - 1;
    new_y = robot.ypos - 1;
end

reachable_action_space = [robot.xpos*ones(4,1), robot.ypos*ones(4,1), possible_orientations'];
[output] = checkifSafe(new_x, new_y, MapParameters);

if output ==  1,
    reachable_action_space = [new_x, new_y, robot.orientation; reachable_action_space];
end

reachable_action_space_silica = reachable_action_space;


rand_opt = 1;

if rand_opt == 0,
    row_num = round(rand*size(reachable_action_space,1));
    if row_num == 0,
        row_num = 1;
    end
    reachable_action_space = reachable_action_space(row_num, :);
    
    row_num = round(rand*size(reachable_action_space_silica,1));
    if row_num == 0,
        row_num = 1;
    end
    reachable_action_space_silica = reachable_action_space_silica(row_num, :);
    
end

end


