function [ utility ] = getUtility( obs_vect, belief_space )
%reads in the probability over the expected observations and the current
%belief space

%calculates the expected change in belief space and assigns a relevant
%utlity value 
utility = sum(obs_vect(:,1));


end

