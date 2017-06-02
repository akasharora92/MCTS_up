%%generates 50 random maps and saves them in a mat file

%number of different location regions
num_x = 4;
num_y = 4;

%number of location per grid
grid_size = 8;


for i = 1:50
    %generate random map
    location_ground_truth = randi([1 3],num_x);
    location_ground_truth = kron(location_ground_truth,ones(grid_size));
    figure; imagesc(location_ground_truth);

    [feature_map_1, feature_map_2, feature_map_3, rock_map, silica_map] = generateRockMap(location_ground_truth, DomainKnowledge_true);

    fullMap = {location_ground_truth, feature_map_1, feature_map_2, feature_map_3, rock_map, silica_map};

    mapName = ['map_' num2str(i) '.mat'];
    save(mapName, 'fullMap');
end