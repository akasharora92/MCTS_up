function [ centre_coords, new_FOV ] = getVisibleCells_silica(robot_pos, robot_orientation, sensor_FOV, MapParameters )
% Returns all currently visible cells to the rover
% works well

%sensor_FOV is a 4 element vector with co-ordinates for the four corners of
%the FOV. 
%robot position and orientation are the desired co-ordinates we want the
%robot to be at

robot_pos = [20*robot_pos(1), 20*robot_pos(2)];

%rotation matrix
R1 = [cosd(robot_orientation) -sind(robot_orientation); sind(robot_orientation) cosd(robot_orientation)]; 

%translation matrix
x_trans = robot_pos(1).*ones(1,4) ;
y_trans = robot_pos(2).*ones(1,4) ;
trans_mat = [x_trans; y_trans];

%transformed
new_FOV = R1*sensor_FOV + trans_mat;

centre_coords = mean(new_FOV, 2);

%check if the cells fall within the map boundaries
%%column < 0
if (centre_coords(1) <= 0 || centre_coords(1) > MapParameters.xsize)
    centre_coords = [];
elseif (centre_coords(2) <= 0 || centre_coords(2) > MapParameters.ysize)
    centre_coords = [];
end



