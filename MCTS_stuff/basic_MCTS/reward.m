%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Example reward function
%
% Graeme Best, ACFR, University of Sydney, Oct 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function r = reward(sequence)

    % reward for each action that has index 1 greater than previous action
    % i.e. 1,2,3,4,5... would give maximum reward
    r = sum( sequence(2:end) == sequence(1:end-1)+1 );
    
    % normalise to [ 0, 1]
    % necessary for UCT to work properly
    max_reward = length(sequence) - 1;
    r = r / max_reward;

end