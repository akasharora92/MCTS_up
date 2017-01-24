%convert location maps into entropy for visualisation

location_entropy = zeros(size(BeliefMaps.Location));
min_entropy = 100;

for i=1:size(location_entropy,1),
    for j=1:size(location_entropy,2),
        prob_dist = BeliefMaps.Location{i,j};
        entropy = abs(sum(prob_dist.*log(prob_dist)));
        location_entropy(i,j) = entropy;
        if entropy < min_entropy,
            min_entropy = entropy;
            min_i = i;
            min_j = j;
        end
    end
end

figure; imagesc(location_entropy);
figure; imagesc(robot.visibility);