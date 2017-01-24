function [ visible_cells, new_FOV ] = getVisibleCells(robot_pos, robot_orientation, sensor_FOV, MapParameters )
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

reverse_or = -1*robot_orientation;
R2 = [cosd(reverse_or) -sind(reverse_or); sind(reverse_or) cosd(reverse_or)]; 
trans_FOV = R2*new_FOV;

%---------------------------
 
x_min = min(trans_FOV(1,:));
x_max = max(trans_FOV(1,:));
y_min = min(trans_FOV(2,:));
y_max = max(trans_FOV(2,:));

visible_cells_x = (x_min:0.5:x_max);
visible_cells_y = (y_min:0.5:y_max);
%disp(visible_cells_x)
%disp(visible_cells_y)

visible_cells_x1 = repmat(visible_cells_x,1,length(visible_cells_y));
visible_cells_x1 = visible_cells_x1(:)';
%disp(visible_cells_x1)

visible_cells_y1 = repmat(visible_cells_y', 1, length(visible_cells_x))';
visible_cells_y1 = visible_cells_y1(:)';
%disp(visible_cells_y1)

visible_cells = [visible_cells_x1; visible_cells_y1];

visible_cells = round(R1*visible_cells)';
visible_cells = unique(visible_cells, 'rows');

%check if the cells fall within the map boundaries
%%column < 0
visible_cells(any(visible_cells<=0,2),:) = [];
%%row < 0
visible_cells(any(visible_cells<=0,1),:) = [];

%%row 
visible_cells(visible_cells(:,1)>MapParameters.xsize,:) = [];
%%column
visible_cells(visible_cells(:,2)>MapParameters.ysize,:) = [];


