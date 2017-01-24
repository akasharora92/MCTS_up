%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A basic implementation of MCTS
% call this function with main.m
%
% Graeme Best, ACFR, University of Sydney, Oct 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [ solution, root, list_of_all_nodes, winner ] = mcts( action_set, budget, max_iterations )

% Create the root node
start_sequence = action_set(1); % this action is picked first always
unpicked_children = action_set; % actions that can be picked next
root = tree_node([], start_sequence, cost(start_sequence), unpicked_children);

% for debugging only - store all nodes in a list
list_of_all_nodes(1) = root;

% Main loop
for iter = 1:max_iterations
    
    disp(['iteration ', num2str(iter)]);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % SELECTION and EXPANSION
    % a new node will get added to the tree
    % current = this new node
    current = root;
    while true
        % check if there are any children to be added
        if ~isempty(current.unpicked_children)
            % pick one of the children that haven't been added
            % just pick the first one...
            child_index = 1;
            child_vertex = current.unpicked_children(child_index);

            % remove this child from the unpicked list
            current.unpicked_children = current.unpicked_children([1:child_index-1, child_index+1:end]);

            % set up new child node
            new_sequence = [current.sequence, child_vertex];
            new_budget_left = budget - cost(new_sequence);
            new_unpicked_children = action_set; % assume all actions are feasible
            
            % remove overbudget children
            flag_keep = true(length(new_unpicked_children),1);
            for i = 1:length(new_unpicked_children)
                new_new_sequence = [new_sequence, new_unpicked_children(i)];
                if cost(new_new_sequence) <= budget
                    flag_keep(i) = true;
                else
                    flag_keep(i) = false;
                end
            end
            new_unpicked_children = new_unpicked_children(flag_keep);

            % add this child node to the tree
            new_child_node = tree_node(current, new_sequence, new_budget_left, new_unpicked_children);
            current.children(end+1) = new_child_node;
            current = new_child_node;
            
            % for debugging only - add new node to list
            list_of_all_nodes(end+1) = new_child_node;

            break;

        else
            % all children have been added, therefore pick one to
            % recurse using UCT policy

            if(isempty(current.children))
                % reached planning horizon, just do this node again
                break;
            end

            % get the UCB score for all children
            child_f_score = zeros(length(current.children),1);
            for i = 1:length(child_f_score)
                % upper confidence bounds
                child_f_score(i) = current.children(i).average_evaluation_score + sqrt( (2 * log( current.num_updates ) ) / ( current.children(i).num_updates ) );
            end
            
            % choose the child that maximises the UCB
            [~, child_chosen_idx] = max(child_f_score);
            current = current.children(child_chosen_idx);
        end
    end
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % ROLLOUT
    % do a rollout from new node to the budget
    % and evaluate the reward
    sequence = rollout(current.sequence, action_set, budget);
    rollout_reward = reward(sequence);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % BACK-PROPAGATE
    % update stats of all nodes from current back to root node
    parent = current;
    while ~isempty(parent)
        % incremental update to the average
        parent.average_evaluation_score = (parent.average_evaluation_score * parent.num_updates + rollout_reward)/( parent.num_updates + 1);
        
        % keep track of number of visits
        parent.num_updates = parent.num_updates + 1;
        
        % recurse up the tree
        parent = parent.parent;
    end
    
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SOLUTION
% calculate best solution so far
% could be selected in many possible ways...
% recursively select child with highest average reward
current = root;
while ~isempty(current.children)
    
    % get the average reward for all children
    child_reward = zeros(length(current.children),1);
    for i = 1:length(child_reward)
        % upper confidence bounds
        child_reward(i) = current.children(i).average_evaluation_score;
    end

    % choose child with max reward
    [~, child_chosen_idx] = max(child_reward);
    current = current.children(child_chosen_idx);
end

solution = current.sequence;
winner = current;

end

