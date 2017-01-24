[x,y] = find(~cellfun(@isempty,robot.obshistory));

for i = 1:size(x,1),
    obs_cell = robot.obshistory{x(i), y(i)};
    disp(i);
    disp(obs_cell);
    rock_dist = BeliefMaps.Rock{x(i), y(i)};
    disp(rock_dist);
    disp(BeliefMaps.F1{x(i), y(i)});
    disp(BeliefMaps.F2{x(i), y(i)});
    disp(BeliefMaps.F3{x(i), y(i)});
    
end