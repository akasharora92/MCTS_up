function [ infoGain_tot, robot_current ] = reward_sequence(state_sequence, BeliefMaps, robot_startstate, sensor, DomainKnowledge, MapParameters, action_path)
%This function takes the robot position and action sequence and simulates
%belief updates. The output is total information gained

%for each sequence
%extract the robot position, orientation and sensor to use
%if local- run belief space silica
%if remote- sample and observation and run belief space normal
%repeat until the end of state sequence is reached and output the infogain
%(it can just be summed over the sequence)

entropy_tot = 0;
%find current entropy of location cells
for i=1:MapParameters.l_rows,
    for j=1:MapParameters.l_cols,
        prob_dist = BeliefMaps.Location{i,j};
        entropy = abs(sum(prob_dist.*log(prob_dist)));
        entropy_tot = entropy_tot + entropy;
        
    end
end


%aim for 0.05s evaluation
infoGain_tot = 0;
repeatflag = 0;
robot_current = robot_startstate;
discount_factor = 1;

for i = 1:size(state_sequence,1),
    robot_current.xpos = state_sequence(i,1);
    robot_current.ypos = state_sequence(i,2);
    robot_current.orientation = state_sequence(i,3);
    sensing_mode = state_sequence(i,4);
    
    for j = 1:length(action_path),
        if action_path(j,1) == 0, %we have exhausted the previous actions
            repeatflag = 0;
            break
        elseif action_path(j,:) == state_sequence(i,:), %current state matches the previous actions
            repeatflag = 1;
            break
        end
    end
    
    if repeatflag == 0,
        if sensing_mode == 0,
            %remote sense
            %get predicted observation
            %simulated belief update and get info gain and new belief
            [BeliefMaps, infoGain, robot_current] = simulate_BeliefUpdate(BeliefMaps, sensor, DomainKnowledge, MapParameters, robot_current, i);
            
        else
            %local sense
            %get predicted observation and simulate belief update and get info
            %gain and new belief
            [BeliefMaps, infoGain, robot_current] = simulate_BeliefUpdate_silica(BeliefMaps, sensor, DomainKnowledge, MapParameters, robot_current, i);
            
        end
        
    else
        infoGain = 0;
    end
    
    infoGain_tot = infoGain_tot + infoGain*(discount_factor^i);
    
end

prob_dist = [1/3 1/3 1/3];
entropy = abs(sum(prob_dist.*log(prob_dist)));
entropy_init = MapParameters.l_cols*MapParameters.l_rows*entropy;

infoGain_tot = 1- ((entropy_tot - infoGain_tot)/entropy_init);
end

