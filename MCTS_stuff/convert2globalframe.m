function [ obs_location_trans] = convert2globalframe( obs_location )
%converts the continuos observation into the discrete world environment

location_vect = obs_location(:,5:6);

%translate the frame
location_vect = location_vect + [0,5]

%scale to discrete grid


%round the number to nearest cell


end

