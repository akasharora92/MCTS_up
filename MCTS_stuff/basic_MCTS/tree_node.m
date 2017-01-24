%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A node of the search tree for MCTS
% no methods here - just holds the variables for a node
% See mcts.m
%
% Graeme Best, ACFR, University of Sydney, Oct 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef tree_node < handle
    
    properties
        
        % Tree properties
        parent                  % backpointer
        children                % list of tree_histories_node
        node_id                 % unique identify for node
        
        x_pos %x and y position in space for the node
        y_pos
        robot_or %absolute orientation of the robot
        
        unpicked_children       % actions for children that could come next
                                % these children are not in sequence or children,
                                % but are within budget if added to the
                                % sequence
        
        % Node properties
        sequence                % sequence of locations up until this location - indices of the vertex matrix
        budget                  % how much budget was used up to and including this vertex
        sense_mode
        
        % Properties estimated by the algorithm (converges to optimal)
        average_evaluation_score
        num_updates             % increments when node is visited
        
    end
    
    methods
        function self = tree_node(parent, sequence, budget, unpicked_children, x_pos, y_pos, robot_or, sense_mode)
            self.num_updates = 0;
            self.average_evaluation_score = 0;
            self.unpicked_children = unpicked_children;
            self.parent = parent;
            self.sequence = sequence;
            self.children = tree_node.empty();
            
            self.x_pos = x_pos;
            self.y_pos = y_pos;
            self.robot_or = robot_or;

            self.budget = budget;
            self.sense_mode = sense_mode;
        end
    end
    
end
