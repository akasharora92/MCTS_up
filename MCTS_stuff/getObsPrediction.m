function [ obs_predict, visible_cells ] = getObsPrediction( robot_pos, robot_orientation, BeliefMaps, sensor )
%outputs a probability vector of observations at each visible cell

%predict which cells will be seen
[visible_cells, ~ ] = getVisibleCells(robot_pos, robot_orientation, sensor.FOV.coords );

%vector of probabilities of taking each observation for each visible cell.
%one column for each value obs can take

obs_predict = ones(size(visible_cells,1), 3);


for i = 1:size(visible_cells,1),
    f_prob1 = BeliefMaps.F1{visible_cells(i,1), visible_cells(i,2)};
    f_prob2 = BeliefMaps.F2{visible_cells(i,1), visible_cells(i,2)};
    f_prob3 = BeliefMaps.F3{visible_cells(i,1), visible_cells(i,2)};
    
    p_z1 = sensor.noise_f1*f_prob1';
    p_z2 = sensor.noise_f1*f_prob2';
    p_z3 = sensor.noise_f1*f_prob3';
    
    %only considering z1 for now
    obs_predict(i,1) = p_z1(1);
    obs_predict(i,2) = p_z1(2);
    obs_predict(i,3) = p_z1(3);
    
    obs_predict(i,4) = p_z2(1);
    obs_predict(i,5) = p_z2(2);
    obs_predict(i,6) = p_z2(3);
    
    obs_predict(i,7) = p_z3(1);
    obs_predict(i,8) = p_z3(2);
    obs_predict(i,9) = p_z3(3);
    
end

end

