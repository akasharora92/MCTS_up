function [info_gain] = getInfoGainFast(location_new, location_original)
%calculates the change in entropy

location_new_sum = 0;
location_old_sum = 0;

for i=1:500,
    for j=1:500,
        location_new_ent = abs(sum(location_new{i,j}.*log(location_new{i,j})));
        location_new_sum = location_new_ent + location_new_sum;
        
        location_old_ent = abs(sum(location_original{i,j}.*log(location_original{i,j})));
        location_old_sum = location_old_ent + location_old_sum;
    end
end

info_gain = location_old_sum - location_new_sum;

end

