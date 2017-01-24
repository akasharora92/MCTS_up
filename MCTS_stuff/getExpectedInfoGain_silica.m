function [ infoGain_ave, infoGain1, infoGain2, infoGain3 ] = getExpectedInfoGain_silica(robot_pos, robot_orientation, BeliefMaps, sensor, DomainKnowledge, MapParameters, robot)
%calculates the expected information gained from an action through sampling

%extract the cells we expect to see from the action
[visible_cells, ~] = getVisibleCells_silica(robot_pos, robot_orientation, sensor.FOV.coords, MapParameters);

infoGain_ave = 0;
infoGain1 = 0;
infoGain2 = 0;
infoGain3 = 0;

if isempty(visible_cells)
    return
end

visible_cells(1) = round(visible_cells(1)/20);
visible_cells(2) = round(visible_cells(2)/20);

if visible_cells(1) == 0
    visible_cells(1) = 1;
end

if visible_cells(2) == 0
    visible_cells(2) = 1;
end


%infoGain_runningave = [];

% sampleSize = 100;
% %infoGain_tot1 = 0;
% infoGain_tot2 = 0;
% 
% %info_gain_comp = [];
% for k=1:sampleSize,
%     obs_vect = zeros(1, 6);
%     p_sdistribution = BeliefMaps.Silica{visible_cells(1), visible_cells(2)};
%     
%     %sample from probability distribution
%     sample_obs = mnrnd(1, p_sdistribution);
%     obs_vect(1) = find(sample_obs == 1);
%     
%     obs_vect(4) = 1;
%     obs_vect(5) = visible_cells(1);
%     obs_vect(6) = visible_cells(2);
%     
%     [~, ~, infoGain2] = updateBeliefSpace_approx(BeliefMaps, obs_vect, robot, sensor, DomainKnowledge, MapParameters);
%     
%     infoGain_tot2 = infoGain2 + infoGain_tot2;
%     
%     %infoGain_runningave = [infoGain_runningave; infoGain_tot2/k];
%     
%     
% end

%figure; plot(infoGain_runningave);
% pause(0.02);
% infoGain_ave = infoGain_tot2/sampleSize;


%save info_gain_comp;
%figure; scatter(info_gain_comp(:,1), info_gain_comp(:,2));

obs_vect = zeros(1, 6);
p_sdistribution = BeliefMaps.Silica{visible_cells(1), visible_cells(2)};

%sample from probability distribution
obs_vect(1) = 1;
obs_vect(4) = 1;
obs_vect(5) = visible_cells(1);
obs_vect(6) = visible_cells(2);
[~, ~, infoGain1] = updateBeliefSpace_approx(BeliefMaps, obs_vect, robot, sensor, DomainKnowledge, MapParameters);

obs_vect(1) = 2;
[~, ~, infoGain2] = updateBeliefSpace_approx(BeliefMaps, obs_vect, robot, sensor, DomainKnowledge, MapParameters);

obs_vect(1) = 3;
[~, ~, infoGain3] = updateBeliefSpace_approx(BeliefMaps, obs_vect, robot, sensor, DomainKnowledge, MapParameters);

infoGain_ave = p_sdistribution(1)*infoGain1 + p_sdistribution(2)*infoGain2 + p_sdistribution(3)*infoGain3;
%disp(['With weights:',num2str(infoGain_ave)]);

% if infoGain_ave < 0,
%     disp('Negative results!!')
%     disp([num2str(robot_pos), ' ',num2str(robot_orientation)])
% end

end








