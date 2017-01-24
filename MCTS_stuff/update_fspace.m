function BeliefMaps = update_fspace(obs, BeliefMaps, sensor)
%updates the feature space based on the observations and the sensor model

%obs - single observation taken at a specific location. 1st element = obs.
%2nd element = location of observation

obs_f1 = obs(1);
obs_f2 = obs(2);
obs_f3 = obs(3);
loc_x = obs(5);
loc_y = obs(6);

prior_F1 = BeliefMaps.F1{loc_x, loc_y};
prior_F2 = BeliefMaps.F2{loc_x, loc_y};
prior_F3 = BeliefMaps.F3{loc_x, loc_y};

%Feature 1 estimate
sensor_model = sensor.noise_f1(obs_f1,:);
prob_zf = sensor_model'.*prior_F1';
prob_zf = (1/sum(prob_zf)).*prob_zf;
BeliefMaps.F1{loc_x, loc_y} = prob_zf';

%Feature 2 estimate
sensor_model = sensor.noise_f1(obs_f2,:);
prob_zf = sensor_model'.*prior_F2';
prob_zf = (1/sum(prob_zf)).*prob_zf;
BeliefMaps.F2{loc_x, loc_y} = prob_zf';

%Feature 3 estimate
sensor_model = sensor.noise_f1(obs_f3,:);
prob_zf = sensor_model'.*prior_F3';
prob_zf = (1/sum(prob_zf)).*prob_zf;
BeliefMaps.F3{loc_x, loc_y} = prob_zf';

%if feature estimate is below a threshold there is probably no rock there-
%make it zero for computational efficiency

end

