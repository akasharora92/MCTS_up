function [entropy_sum] = getJointEntropy( BeliefMaps_location, xpos, ypos)
%Calculates the entropy of the Belief Map within a radius of 400 cells
entropy_sum = 0;

for i=1:size(BeliefMaps_location,1),
    for j=1:size(BeliefMaps_location,2),
        prob_dist = BeliefMaps_location{i,j};
        entropy = abs(sum(prob_dist.*log(prob_dist)));
        entropy_sum = entropy + entropy_sum;
    end
end

end

