%this function generates a random map of size x,y

%number of different location regions
num_x = 5;
num_y = 5;

%number of location per grid
grid_size = 8;

location_ground_truth = randi([1 3],5);
location_ground_truth = kron(location_ground_truth,ones(8));

[feature_map_1, feature_map_2, feature_map_3, rock_map, silica_map] = generateRockMap(location_ground_truth, DomainKnowledge);