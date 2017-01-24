function [ infoGain_ave, infoGain_std ] = getExpectedInfoGain(robot_pos, robot_orientation, BeliefMaps, sensor, DomainKnowledge, MapParameters, robot)
%calculates the expected information gained from an action through sampling

%extract the cells we expect to see from the action
[visible_cells, ~] = getVisibleCells(robot_pos, robot_orientation, sensor.FOV.coords, MapParameters);

infoGain_ave = 0;
infoGain_std = 0;

if isempty(visible_cells)
    return
end

infoGain_runningave = [];
%sample 5 particles for each action
sampleSize = 20;
%infoGain_tot1 = 0;
infoGain_tot2 = 0;

infoGain_vect = zeros(sampleSize,1);
%info_gain_comp = [];
for k=1:sampleSize,
    obs_vect = zeros(length(visible_cells), 6);
    for i=1:length(visible_cells),
        rand_sample = rand;
        
        %check which of the cells have been observed already
        if robot.visibility(visible_cells(i,1),visible_cells(i,2)) == 1,
            %rock wasnt there before so we shouldnt see it again
            continue;
            
        elseif ((robot.visibility(visible_cells(i,1),visible_cells(i,2)) == 2) || (rand_sample < 0.0015)), %there is a rock already there or its there in the sample
            %if there was a rock we expect to see a rock again
            f_prob1 = BeliefMaps.F1{visible_cells(i,1), visible_cells(i,2)};
            f_prob2 = BeliefMaps.F2{visible_cells(i,1), visible_cells(i,2)};
            f_prob3 = BeliefMaps.F3{visible_cells(i,1), visible_cells(i,2)};
            
            p_z1 = sensor.noise_f1*f_prob1';
            p_z2 = sensor.noise_f1*f_prob2';
            p_z3 = sensor.noise_f1*f_prob3';
            
            %sample from probability distribution
            sample_obs = mnrnd(1, p_z1);
            obs_vect(i,1) = find(sample_obs == 1);
            
            sample_obs = mnrnd(1, p_z2);
            obs_vect(i,2) = find(sample_obs == 1);
            
            sample_obs = mnrnd(1, p_z3);
            obs_vect(i,3) = find(sample_obs == 1);
            obs_vect(i,4) = 0;
            obs_vect(i,5) = visible_cells(i,1);
            obs_vect(i,6) = visible_cells(i,2);
        else
            continue;
        end
    end
    
    obs_vect(obs_vect(:,1) == 0, :) = [];
    
%     if(size(obs_vect,1) > 8)
%         disp('Observations are large!')
%     end
    
    if isempty(obs_vect),
        continue
    end
    %disp(obs_vect)
    %tic
    [~, ~, infoGain2] = updateBeliefSpace_approx(BeliefMaps, obs_vect, robot, sensor, DomainKnowledge, MapParameters);
    %toc
    infoGain_vect(k) = infoGain2;
%     if infoGain2 < 0,
%         disp('Below 0')
%         disp(obs_vect)        
%     end
%     
%     if infoGain2 > 0,
%         disp('Above 0')
%         disp(obs_vect)
%     end
    
    %tic
    %[~, ~, infoGain2] = updateBeliefSpace_fast(BeliefMaps, obs_vect, robot, sensor, DomainKnowledge, MapParameters);
    %toc
    
    %info_gain_comp = [info_gain_comp; infoGain1, infoGain2];
    
    %[location_entropy_final] = getJointEntropy(BeliefMaps2.Location, robot_pos(1), robot_pos(2));
    %toc
    %infoGain = location_entropy_init - location_entropy_final;
    %infoGain_tot1 = infoGain1 + infoGain_tot1;
    infoGain_tot2 = infoGain2 + infoGain_tot2;
   % disp(k);
   %infoGain_runningave = [infoGain_runningave; infoGain_tot2/k];
    
    %disp(infoGain1)
    %disp(infoGain2)
    
end

%figure; plot(infoGain_runningave);
pause(0.02);
infoGain_ave = infoGain_tot2/sampleSize;
infoGain_std = std(infoGain_vect);
%disp(infoGain_std);
%save info_gain_comp;
%figure; scatter(info_gain_comp(:,1), info_gain_comp(:,2));

end






