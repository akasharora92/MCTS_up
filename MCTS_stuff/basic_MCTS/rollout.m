%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Example rollout function
%
% Graeme Best, ACFR, University of Sydney, Oct 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function sequence = rollout(subsequence, action_set, budget)

    % pick random actions until budget is exhausted
    sequence = subsequence;
    while cost(sequence) < budget
        r = randi(length(action_set));
        sequence(end+1) = action_set(r);
    end

end