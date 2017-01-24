function [ obs_vect ] = generate_obs(feature_map_1,feature_map_2, feature_map_3, visible_cells, sensor)
%based on the simulated map and the sensor model a sample set of
%observations are generated
obs_vect = zeros(length(visible_cells), 6);
obs_vect(:,4) = 0;

for i = 1:length(visible_cells),
    %check each visible cell to see if a feature exists or not
    feature_1 = feature_map_1(visible_cells(i,1), visible_cells(i,2));
    if (feature_1 ~= 0),
        %disp(visible_cells(i,:));
        
        %observation 1
        sensor_model = sensor.noise_f1(:,feature_1);
        sample_obs = mnrnd(1, sensor_model);
        obs_vect(i,1) = find(sample_obs == 1);
        
        %observation 2
        feature_2 = feature_map_2(visible_cells(i,1), visible_cells(i,2));
        sensor_model = sensor.noise_f1(:,feature_2);
        sample_obs = mnrnd(1, sensor_model);
        obs_vect(i,2) = find(sample_obs == 1);
        
        %observation 3
        feature_3 = feature_map_3(visible_cells(i,1), visible_cells(i,2));
        sensor_model = sensor.noise_f1(:,feature_3);
        sample_obs = mnrnd(1, sensor_model);
        obs_vect(i,3) = find(sample_obs == 1);
        
        obs_vect(i,5) = visible_cells(i,1);
        obs_vect(i,6) = visible_cells(i,2);
        
    else
        %obs_vect(i,:) = [];
        continue;
    end
    
end

%clear empty observations
obs_vect(obs_vect(:,1) == 0, :) = [];

end

