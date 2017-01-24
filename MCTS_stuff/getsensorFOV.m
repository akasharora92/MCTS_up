function [FOV] = getsensorFOV(sensor)
%calculates the rectangular co-ordinates of the robot's FOV

top_left = [sensor.x_trans - 0.5*sensor.FOV.width; sensor.y_trans + 0.5*sensor.FOV.height];
top_right = top_left + [sensor.FOV.width; 0];
bottom_right = top_left + [sensor.FOV.width; -sensor.FOV.height];
bottom_left = top_left + [0; -sensor.FOV.height];

FOV = [top_left top_right bottom_right bottom_left];

end

