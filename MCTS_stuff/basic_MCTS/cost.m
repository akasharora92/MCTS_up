%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cost function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [c] = cost(sequence)

cost_remote = 1;
cost_local = 8;

c = 0;
if isempty(sequence),
    return
end

for i=1:length(sequence),
    if sequence(i) == 0,
        c = c + cost_remote;
    else
        c = c + cost_local;
    end
end

end