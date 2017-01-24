
%remote sense
robot_pos = [20 10];
robot_orientation = 45;

%%iterate over reachable action space
%arm/camera position fixed- robot can only choose x,y,orientation and
%choice of sensor
%

%predict which cells will be seen
[visible_cells, new_FOV ] = getVisibleCells(robot_pos, robot_orientation, sensor.FOV.coords );

%plot to make sure algorithm works
% figure; scatter(visible_cells(:,1), visible_cells(:,2));
% hold on
% scatter(new_FOV(1,:), new_FOV(2,:));

%predict what may be seen based on the belief on feature space and the
%sensor model

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
    
end

%%we cant calculate the utility for every possible observation so we need
%%to either average or be optimistic about what we might see
%%EU(action) = integrate(P(Z|a).U(Z).dZ)
[ utility ] = getExpectedUtility(obs_predict, BeliefMaps);

%%iterate over action space- start with greedy and expand to receding
%%horizon