%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Function for plotting an MCTS tree
%
% Graeme Best, ACFR, University of Sydney, Oct 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function plot_tree(list_of_all_nodes, winner, num_vertices, fig, UCT)

    % if UCT is 1 - plot UCT scores as colours
    % if UCT is 0 - plot average rewards as colours

    figure(fig);
    clf;
    hold on;
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % PLOT TRAVERSED TREE

    l = list_of_all_nodes;
    
    % find the min and max reward for scaling colour
    r_min = 1;
    r_max = 0;
    for i = 1:length(l)
        n = l(i);
        
        if UCT == 0
        	r = n.average_evaluation_score;
            
            if r < r_min
                r_min = r;
            end
            if r > r_max
                r_max = r;
            end
        else
            if ~isempty(n.parent)
                r = n.average_evaluation_score + sqrt( (2 * log( n.parent.num_updates ) ) / ( n.num_updates ) );

                if r < r_min
                    r_min = r;
                end
                if r > r_max
                    r_max = r;
                end
            end
        end
    end

    for i = 1:length(l)
        
        n = l(i);
        
        % Get my position in the tree
        my_depth = length(n.sequence);
        my_position = get_position(n.sequence,num_vertices);
        
        

        % Plot edge to parent
        if ~isempty(n.parent)
            
            w = 2;
%           col = [0 0.7 0];
            if UCT == 0
                r = n.average_evaluation_score;
            else
                r = n.average_evaluation_score + sqrt( (2 * log( n.parent.num_updates ) ) / ( n.num_updates ) );
            end
            r = (r - r_min)/(r_max-r_min);
            col = [0 r 0]; % colour proportional to reward 
        
        
            parent_depth = length(n.parent.sequence);
            parent_position = get_position(n.parent.sequence,num_vertices);
            
            edge_col = col;
            plot([my_position, parent_position], -[my_depth, parent_depth], '-k', 'Color', edge_col, 'LineWidth',w);

            if n == winner
                h = plot(my_position, -my_depth, 'or', 'LineWidth', 5, 'MarkerSize', 8);
            end
        end
    end
    
    uistack(h,'top');
    
    axis tight
    axis off
    set(gcf,'color','w');
    
    drawnow

end

function pos = get_position(seq, n)
    %     position = seq(end);

    
    
    pos = - scramble(seq(1), n);
    eps = 0;%0.1;
    maxn = 6;
    
    for i = 1:length(seq)
        pos = max(((-eps/maxn)*(i-1)+1+eps),1)*(pos + scramble(seq(i), n) * (n)^(-i+1)) ;
    end
end



function i = scramble(i, n)
%     i = mod((i + floor(n/2) ), n);
    i = -(i - ceil(n/2)) / n;
end

function pruned = check_pruned(root, seq)

    pruned = 0;
    
    count = 2;
    
    current = root;
    
    while count <= length(seq)
        
        % if a subsequence is pruned, this node is pruned
        if current.pruned
            pruned = 1;
            return;
        end
        
        % find the next child
        looking_for = seq(count);
        c_next = [];
        for c = current.children
            if c.sequence(end) == looking_for
                c_next = c;
                break;
            end
        end
        
        % reached end of tree?
        if isempty(c_next)
            
            for v = current.pruned_node_vertices
                if v == looking_for
                    % this sequence was previously a child of this node but
                    % was pruned 
                    pruned = 1;
                    return;
                end
            end
            
            return;
        end
        
        count = count + 1;
        current = c_next;
        
    end

%     for i = 1:length(list_of_all_nodes)
%         n = list_of_all_nodes(i);
% 
%         % check if node has been pruned
%         if n.pruned == 1
%             % if so, check if it is a subsequence of seq
%             if length(n.sequence) <= length(seq)
%                 if sum(n.sequence == seq(1:length(n.sequence))) == length(n.sequence)
%                     pruned = 1;
%                 end
%             end
%         end
%     end
end

