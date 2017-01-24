function [action_space] = getActionSpace(robot, MapParameters)
%get a set of possible actions based on the robot's current position and
%restrictions on the map
action_space_mult = 20; 

%possible_orientations = [0:45.:315];
possible_orientations = [0:90.:270];

possible_x_pos = [-1:1].*action_space_mult;
possible_x_pos = possible_x_pos + robot.xpos;

possible_y_pos = [-1:1].*action_space_mult;
possible_y_pos = possible_y_pos + robot.ypos;

action_space = [];


for i = 1:length(possible_x_pos),
    for j=1:length(possible_y_pos),
        [ output ] = checkifSafe( possible_x_pos(i), possible_y_pos(j), MapParameters );
        
        if output == 0,
            continue
        end
        
        %check if parts of the map are collision free
        %to do
        
        
        for k=1:length(possible_orientations),
            action = [possible_x_pos(i) possible_y_pos(j), possible_orientations(k)];
            action_space = [action_space; action];
        end
    end
end

end

