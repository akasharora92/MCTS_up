function [feature_map_1, feature_map_2, feature_map_3, rock_map, silica_map] = generateRockMap(location_ground_truth, DomainKnowledge)
%creates a random rock map given the underlying location type and the
%domain knowledge parameters

%Generate ground truth silica map
silica_map = zeros(size(location_ground_truth));
for i=1:size(location_ground_truth,1),
    for j=1:size(location_ground_truth,2),
        s_likelihood = DomainKnowledge.theta_sl(:, location_ground_truth(i,j));
        s_sample = mnrnd(1, s_likelihood);
        silica_map(i,j) = find(s_sample == 1);                
    end
end

location_ground_truth = kron(location_ground_truth,ones(20));

rock_map = zeros(size(location_ground_truth));
feature_map_1 = zeros(size(location_ground_truth));
feature_map_2 = zeros(size(location_ground_truth));
feature_map_3 = zeros(size(location_ground_truth));

ave_rock_freq = 0.0015; 

for i = 1:size(location_ground_truth, 1),
    for j = 1:size(location_ground_truth, 2),
        %disp(i);
        %disp(j);
        rand_Num = rand;
        if rand_Num < ave_rock_freq,
            %generate a rock based on the domain knowledge parameters
            location_type = location_ground_truth(i,j);
            rock_likelihood_mat = DomainKnowledge.theta_rl(:, location_type)';
            
            %sample from categorical distribution
            rock_sample = mnrnd(1, rock_likelihood_mat);
            rock_map(i,j) = find(rock_sample == 1);
            
            %create feature space
            %feature 1
            f_likelihood_mat = DomainKnowledge.theta_fr_1(:, rock_map(i,j));
            %sample
            f_sample = mnrnd(1, f_likelihood_mat);
            feature_map_1(i,j) = find(f_sample == 1);
            
            %feature 2
            f_likelihood_mat = DomainKnowledge.theta_fr_2(:, rock_map(i,j));
            %sample
            f_sample = mnrnd(1, f_likelihood_mat);
            feature_map_2(i,j) = find(f_sample == 1);
            
            %feature 3
            f_likelihood_mat = DomainKnowledge.theta_fr_3(:, rock_map(i,j));
            %sample
            %disp(f_likelihood_mat);
            f_sample = mnrnd(1, f_likelihood_mat);
            %disp(f_sample);
            feature_map_3(i,j) = find(f_sample == 1);           
            
        else
            %do not generate a rock
            continue;
        end
        
    end
end

end

