function [ obs_location ] = camera2world( pixel_obs, robot_location, sensor)
%converts the measurement in the pixel frame to global frame

%pixel_obs is a vector of rock positions detected
%robot_location is a 6 dimensional vector of x,y,z,roll,pitch, yaw
%We only interested in roll, pitch and yaw for now
%convert pixel value to sensor FOV

camera_y_res = 1080;
camera_x_res = 1920;

%change scale of observations- pixel to grid
pixel_obs(:,1) = pixel_obs(:,1).*(sensor.FOV.width/camera_x_res);
pixel_obs(:,2) = pixel_obs(:,2).*(sensor.FOV.height/camera_y_res);

%offset between robot and camera frame- accounting for the offset of the
%axes
robot_translation = 0.5*[sensor.FOV.coords(2,1) + sensor.FOV.coords(2,3)];

R1 = [cosd(90) -sind(90); sind(90) cosd(90)];
translation_offset = [0.5*sensor.FOV.width; - (0.5*sensor.FOV.height + robot_translation)];
local_frame = R1*pixel_obs';
local_frame(1,:) = local_frame(1,:) + translation_offset(1);
local_frame(2,:) = local_frame(2,:) + translation_offset(2);


%convert the local frame to global frame by taking into account the
%robot localisation
robot_orientation = robot_location(6);
robot_pos = [robot_location(1), robot_location(2)];

%convert measurement in metres to grid cell position
robot_pos = robot_pos.*50;


R1 = [cos(robot_orientation) -sin(robot_orientation); sin(robot_orientation) cos(robot_orientation)]; 

%rotate
obs_location = R1*local_frame;
obs_location = obs_location';

%translate
obs_location(:,1) = obs_location(:,1) + robot_pos(1);
obs_location(:,2) = obs_location(:,2) + robot_pos(2);

%localisation has a funny origin point- change to corner of the map
obs_location(:,1) = obs_location(:,1) + 0;
obs_location(:,2) = obs_location(:,2) + 300;

obs_location(:,1:2) = round(obs_location(:,1:2));


end

