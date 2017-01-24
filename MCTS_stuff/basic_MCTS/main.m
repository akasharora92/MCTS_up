
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A basic implementation of MCTS
%
% Graeme Best, ACFR, University of Sydney, Oct 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Run MCTS
action_set = 1:3;      % at each step, agent will pick one of these actions
budget = 10;            % number of actions that can be selected
max_iterations = 1000;  % loops of MCTS
[ solution, root, list_of_all_nodes, winner ] = mcts( action_set, budget, max_iterations );

% Print the result
% note that the optimal solution is (1,2,3,1,2,3,...) up to budget
% the solution length may be less than the budget if tree has not been
% fully explored
% see reward() for how this is defined
display(solution)

% Plot the tree
plot_tree(list_of_all_nodes, winner, length(action_set), 1, 0);
title('average reward');
plot_tree(list_of_all_nodes, winner, length(action_set), 2, 1);
title('UCB value');