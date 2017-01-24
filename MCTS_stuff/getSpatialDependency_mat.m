function [ weighting_matrix ] = getSpatialDependency_mat( location_1, location_2_x, location_2_y )
%calculates the influence of location 1 on location 2
%outputs 1 if perfect correlation and 0 if no correlation
%returns a matrix if given a matrix of locations to evaluate


dist_x = abs(location_2_x - location_1(1));
dist_y = abs(location_2_y - location_1(2));

dist = sqrt(dist_x.^2 + dist_y.^2);

%exponential model
weighting_matrix = exp(-0.015.*dist);


end

