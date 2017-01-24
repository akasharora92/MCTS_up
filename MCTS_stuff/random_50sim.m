%this script runs one trial of a simulation in a fixed environment

tot_num = 50;
num_actions = 50;

ent_data = zeros(num_actions, tot_num);
cost_data = zeros(num_actions, tot_num);
recog_scoredata = zeros(num_actions, tot_num);
unexplored_celldata = zeros(num_actions, tot_num);


tot_budget = 50;

for sim_run = 1:tot_num,
    
    %initialise visibility map
    robot.visibility = zeros(MapParameters.xsize, MapParameters.ysize);
    robot.visibility_silica = zeros(MapParameters.l_rows, MapParameters.l_cols);
    
    %initialise previous obs map
    robot.obshistory = cell(MapParameters.xsize,MapParameters.ysize);
    
    %initialise old message storage (for computational efficiency)
    robot.oldmsgs = cell(MapParameters.xsize,MapParameters.ysize);
    
    %initialise robot mode (remote=0 or local sense=1)
    robot.mode = 0;
    
    robot.xpos = round(rand(1)*(MapParameters.l_rows-10)) + 5; %100 cell padding to prevent rovers starting from the edge
    robot.ypos = round(rand(1)*(MapParameters.l_cols-10)) + 5;
    robot.orientation = 0;
    
    
    %generate random map
    location_ground_truth = randi([1 3],num_x);
    location_ground_truth = kron(location_ground_truth,ones(grid_size));
    figure; imagesc(location_ground_truth);
    
    [feature_map_1, feature_map_2, feature_map_3, rock_map, silica_map] = generateRockMap(location_ground_truth, DomainKnowledge_true);
    
    %clear belief spaces
    %initialise rock and silica belief maps
    for i = 1:numel(BeliefMaps.Location)
        BeliefMaps.Location(i) = {priorLoc};
        BeliefMaps.Silica(i)   = {prior_S'};
    end
    
    %initialise rock and feature belief maps
    for i = 1:numel(BeliefMaps.Rock)
        BeliefMaps.Rock(i)     = {priorRock'};
        BeliefMaps.F1(i)       = {prior_f1'};
        BeliefMaps.F2(i)       = {prior_f2'};
        BeliefMaps.F3(i)       = {prior_f3'};
    end
    
    
    
    cost_remote = 1;
    cost_local = 8;
    endstate = 0;
    
    cost_tot = 0;
    
    actualinfogain_vect = zeros(num_actions,1);
    
    
    action_path = zeros(num_actions, 4);
    
    repeat_flag = 0;
    
    for iterate = 1:num_actions,
        
        leftover_budget = tot_budget - cost_tot;
        
        action_path(iterate,:) = [robot.xpos, robot.ypos, robot.orientation, robot.mode];
        
        
        if robot.mode == 0, %remote sense is to be made
            %get the visible cells
            [visible_cells, new_FOV] = getVisibleCells([robot.xpos, robot.ypos], robot.orientation, sensor.FOV.coords, MapParameters);
            
            %get observation vector based on the map
            [ obs_vect ] = generate_obs(feature_map_1,feature_map_2, feature_map_3, visible_cells, sensor);
            
        else
            [visible_cells, ~] = getVisibleCells_silica([robot.xpos, robot.ypos], robot.orientation, sensor.FOV.coords, MapParameters);
            if isempty(visible_cells),
                break
            end
            visible_cells(1) = round(visible_cells(1)/20);
            visible_cells(2) = round(visible_cells(2)/20);
            
            if visible_cells(1) == 0
                visible_cells(1) = 1;
            end
            
            if visible_cells(2) == 0
                visible_cells(2) = 1;
            end
            
            obs_vect = zeros(1,6);
            obs_vect(1) = silica_map(visible_cells(1), visible_cells(2));
            obs_vect(4) = 1;
            obs_vect(5) = visible_cells(1);
            obs_vect(6) = visible_cells(2);
        end
        
        %update belief space based on the observations
        if repeat_flag == 0,
            [BeliefMaps, robot, infoGain_actual] = updateBeliefSpace(BeliefMaps, obs_vect, robot, sensor, DomainKnowledge, MapParameters);
        else
            infoGain_actual = 0;
        end
        
        repeat_flag = 0;
        
        disp('Actual information gained: ')
        disp(infoGain_actual)
        actualinfogain_vect(iterate) = infoGain_actual;
        
        %plot location_entropy
        location_entropy = zeros(size(BeliefMaps.Location));
        entropy_tot = 0;
        recog_score = 0;
        unexplored_cell_num = 0;
        
        for i=1:size(location_entropy,1),
            for j=1:size(location_entropy,2),
                prob_dist = BeliefMaps.Location{i,j};
                entropy = abs(sum(prob_dist.*log(prob_dist)));
                location_entropy(i,j) = entropy;
                entropy_tot = entropy_tot + entropy;
                
                true_location = location_ground_truth(i,j);
                recog_score = recog_score + prob_dist(true_location);
                
                if (abs(entropy - 1.0986) < 0.01)
                    unexplored_cell_num = unexplored_cell_num + 1;
                end
                
            end
        end
        
        entropy_curve(iterate) = entropy_tot;
        recog_scoredata(iterate) = recog_score;
        subplot(2,2,1), imagesc(location_entropy);
        subplot(2,2,2),imagesc(robot.visibility);
        
        A = BeliefMaps.Location ;
        B1 = cell2mat(arrayfun(@(x)permute(x{:},[3 1 2]),A,'UniformOutput',false));
        subplot(2,2,3), imagesc(B1);
        subplot(2,2,4), imagesc(location_ground_truth);
        
        pause(0.01);
        
        recog_scoredata(iterate, sim_run) = recog_score;
        ent_data(iterate, sim_run) = entropy_curve(iterate);
        cost_data(iterate, sim_run) = cost_tot;
        
        
        
        if cost_tot >= tot_budget,
            disp('Budget exhausted')
            endstate = 1;
        end
        
        if endstate == 1,
            break
        end
        
        
        %get action space of the robot from the current node
        [reachable_action_space, reachable_action_space_silica] = getActionSpace_new(robot, MapParameters);
        
        sensor_select = round(rand(1)); %select remote or local sense
        action_select = round(randi(size(reachable_action_space,1)));
        
        
        if tot_budget - cost_tot < cost_local, %dont allow local sense if budget will not allow
            sensor_select = 0;
        end
        
        if (sensor_select == 0),
            best_action = [reachable_action_space(action_select,:)];
            robot.mode = 0; %best action is a remote sense
        else
            best_action = [reachable_action_space_silica(action_select,:)];
            robot.mode = 1;
        end
        
        
        %select optimal action
        opt_action = best_action;
        
        
        
        %execute optimal action
        robot.xpos = opt_action(1);
        robot.ypos = opt_action(2);
        robot.orientation = opt_action(3);
        state_vect = [robot.xpos, robot.ypos, robot.orientation, robot.mode];
        
        for k=1:size(action_path,1),
            if (action_path(k,:) == state_vect(1,:)), %current state matches the previous actions
                repeat_flag = 1; %we want to set information gain to 0
                break
            end
            
        end
        
        if robot.mode == 1, %local sense made
            cost_tot = cost_tot + cost_local;
            disp('Local sense made')
        else
            cost_tot = cost_tot + cost_remote;
            disp('Remote sense made')
        end
        
        
        
        
        
    end
end
