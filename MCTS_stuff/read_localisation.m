%read localisation text file and convert to matrix
fileID = fopen('localisation_log2.txt','r');
A = textscan(fileID,'%*q %f %f %f %f %f %f','Delimiter',',');
A = cell2mat(A);
fclose(fileID);

%read python file output to get feature space and rock locations
fileID = fopen('fspace5.txt','r');
B = textscan(fileID, '%f');
B = cell2mat(B);
fclose(fileID);

size_A = length(B);
n_features = size_A/5;

f_space = reshape(B, [5, n_features])';
pixel_obs = f_space(:,1:2);
f_space = f_space(:,3:5);

%first observation
robot_location = A(5,:);
[obs_location] = camera2world(pixel_obs, robot_location, sensor);
%give the sensor reasonable values and discretise the location



ones_vect = zeros(n_features, 1);
obs_vect = [f_space ones_vect obs_location];

for i=1:n_features,
    if obs_vect(i,1) < 0.05,
        obs_vect(i,1) = 1;
    elseif obs_vect(i,1) < 0.1,
        obs_vect(i,1) = 2;
    else
        obs_vect(i,1) = 3;
    end
    
    if obs_vect(i,2) < 0.01,
        obs_vect(i,2) = 1;
    elseif obs_vect(i,2) < 0.05,
        obs_vect(i,2) = 2;
    else
        obs_vect(i,2) = 3;
    end
    
    if obs_vect(i,3) < 50,
        obs_vect(i,3) = 1;
    elseif obs_vect(i,3) < 100,
        obs_vect(i,3) = 2;
    else
        obs_vect(i,3) = 3;
    end
end
