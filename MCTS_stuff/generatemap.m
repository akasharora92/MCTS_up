%generate ground truth location map
%3 classes

location_ground_truth = zeros(MapParameters.xsize, MapParameters.ysize);

figure; imagesc(location_ground_truth);

n_location1 = 5;

for i = 1:n_location1,
%label location 1
location_1 = roipoly;
location_ground_truth(location_1) = 1;
imagesc(location_ground_truth);

end

close all;

n_location2 = 5;
figure; imagesc(location_ground_truth);

for i = 1:n_location1,
%label location 1
location_2 = roipoly;
location_ground_truth(location_2) = 2;
imagesc(location_ground_truth);

end

location_ground_truth(location_ground_truth == 0) = 3;

%generate rocks and features
[feature_map_1, feature_map_2, feature_map_3, rock_map] = generateRockMap(location_ground_truth, DomainKnowledge);

% save location_ground_truth;
% save feature_map_1;
% save feature_map_2;
% save feature_map_3;
% save rock_map;

