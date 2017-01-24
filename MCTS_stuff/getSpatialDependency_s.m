function [ weighting ] = getSpatialDependency_s( location_1, location_2 )
%calculates the influence of location 1 on location 2
%outputs 1 if perfect correlation and 0 if no correlation

dist_x = abs(location_1(1) - location_2(1));
dist_y = abs(location_1(2) - location_2(2));

dist = sqrt(dist_x^2 + dist_y^2);

%exponential model
weighting = exp(-0.01.*dist);


end
