%this script attempts to run one decision cycle with continuum

%select arbitary action- for testing only
robot.xpos = 50;
robot.ypos = 50;
robot.orientation = 0;
%robot.mode = 0;

cost_remote = 1;
cost_local = 10;

figure;
hold on;

%convert current position to grid cell position
%get localisation
%call localisation and save to text file
cmd_localisation = 'localisation-get --host 192.168.2.150  --ascii --average=1 > localisation_log.txt &';
[status, cmdout] = system(cmd_localisation);
pause(0.2);

fileID = fopen('localisation_log.txt','r');
A = textscan(fileID,'%*q %f %f %f %f %f %f','Delimiter',',');
A = cell2mat(A);
fclose(fileID);
robot_pos = [A(1), A(2)];
robot_orientation = -A(6);

%subtract origin point
robot_pos = [robot_pos(1) - yard_origin(1), robot_pos(2) - yard_origin(2)];

%multiply by resolution to get values in terms of grid. 50 cells per meter
robot_pos = 50.*robot_pos;

robot_pos = round(robot_pos);

%for plotting purposes rows = y, cols = x
robot_pos = [robot_pos(2), robot_pos(1)];
robot.xpos = robot_pos(1);
robot.ypos = robot_pos(2);

robot.orientation = rad2deg(robot_orientation);
%check if remote or local sense is to be made

if robot.mode == 0,
    disp('Need to make remote sense')
else
    disp('Need to make local sense')
end

prompt = 'Is input ready?';
%call feature extraction script on other computer
x = input(prompt);

%carry out the sensing action and retrive feature space
if robot.mode == 0, %remote sense is to be made
    %get the visible cells
    [visible_cells, new_FOV] = getVisibleCells([robot.xpos, robot.ypos], robot.orientation, sensor.FOV.coords, MapParameters);
    
    %[ obs_vect ] = generate_obs(feature_map_1,feature_map_2, feature_map_3, visible_cells, sensor);
    
    %get image from arm cam and save as image.jpg
    disp('Getting image from arm cam')
    %     command = 'wget -O image.jpg "http://admin:admin@203.9.149.241:50104/snap.cgi"';
    %     [status,cmdout] = system(command);
    
    
    %call feature extraction program
    disp('Extracting features from image');
    %command = call python program >> fspace.txt
    %[status,cmdout] = system(command);
    %we are getting fspace.txt directly from red for now
    
    
    
    
    %get localisation and convert to grid world
    disp('Fusing localisation with observation')
    [obs_vect] = read_localisation(sensor, yard_origin);
    
    
    
    
else
    [visible_cells, ~] = getVisibleCells_silica([robot.xpos, robot.ypos], robot.orientation, sensor.FOV.coords, MapParameters);
    visible_cells(1) = round(visible_cells(1)/20);
    visible_cells(2) = round(visible_cells(2)/20);
    
    if visible_cells(1) == 0
        visible_cells(1) = 1;
    end
    
    if visible_cells(2) == 0
        visible_cells(2) = 1;
    end
    
    obs_vect = zeros(1,6);
    obs_vect(4) = 1;
    obs_vect(5) = visible_cells(1);
    obs_vect(6) = visible_cells(2);
    %robot_orientation
    %run silica feature extracter
    %read python file output to get feature space and rock locations
    fileID = fopen('fspace.txt','r');
    B = textscan(fileID, '%f');
    B = cell2mat(B);
    fclose(fileID);

    obs_vect(1) = B; 
end


%update the belief as usual
disp('Updating belief')
[BeliefMaps, robot, ~] = updateBeliefSpace(BeliefMaps, obs_vect, robot, sensor, DomainKnowledge, MapParameters);

%get action space of the robot
%[reachable_action_space] = getActionSpace(robot, MapParameters);
[reachable_action_space, reachable_action_space_silica] = getActionSpace_new(robot, MapParameters, action_space);


%iterate through the action space to find the best action
%check traversability as well

best_utility = 0;
best_action = [robot.xpos, robot.ypos, robot.orientation];

disp('Attempting to find next best action')
%Evaluate camera sensing actions
for i=1:size(reachable_action_space,1),
    robot_pos = [reachable_action_space(i,1), reachable_action_space(i,2)];
    robot_orientation = reachable_action_space(i,3);
    
    tic
    [utility,~] = getExpectedInfoGain(robot_pos, robot_orientation, BeliefMaps, sensor, DomainKnowledge, MapParameters, robot);
    toc
    
    disp(i)
    disp(utility)
    
    utility = utility/cost_remote;
    
    %incorporate cost
    
    if (utility > best_utility),
        best_utility = utility;
        best_action = [reachable_action_space(i,:)];
        robot.mode = 0; %best action is a remote sense
    end
end


%Evaluate UV sensing actions
for i=1:size(reachable_action_space_silica,1),
    robot_pos = [reachable_action_space_silica(i,1), reachable_action_space_silica(i,2)];
    robot_orientation = reachable_action_space_silica(i,3);
    
    tic
    [utility,~,~,~] = getExpectedInfoGain_silica(robot_pos, robot_orientation, BeliefMaps, sensor, DomainKnowledge, MapParameters, robot);
    toc
    
    disp(i)
    
    utility = utility/cost_local;
    disp(utility)
    %incorporate cost
    
    if (utility > best_utility),
        best_utility = utility;
        best_action = [reachable_action_space_silica(i,:)];
        robot.mode = 1; %optimal action is local sense
    end
end

%plot location_entropy
location_entropy = zeros(size(BeliefMaps.Location));
entropy_tot = 0;

for i=1:size(location_entropy,1),
    for j=1:size(location_entropy,2),
        prob_dist = BeliefMaps.Location{i,j};
        entropy = abs(sum(prob_dist.*log(prob_dist)));
        location_entropy(i,j) = entropy;
        entropy_tot = entropy_tot + entropy;
        %             if entropy < min_entropy,
        %                 min_entropy = entropy;
        %                 min_i = i;
        %                 min_j = j;
        %             end
    end
end

subplot(2,2,1), imagesc(location_entropy);
subplot(2,2,2),imagesc(robot.visibility);

A = BeliefMaps.Location ;
B1 = cell2mat(arrayfun(@(x)permute(x{:},[3 1 2]),A,'UniformOutput',false));
subplot(2,2,3), imagesc(B1);
subplot(2,2,4), imagesc(location_ground_truth);

pause(0.01);

if robot.mode == 0,
    disp('Best action is remote sense')
else
    disp('Best action is local sense')
end

disp('The best action is:')


%select optimal action
opt_action = best_action;


%convert action to a more suitable number for the Mars Yard
[opt_action] = convert_Yardframe(opt_action, yard_origin);

disp(opt_action)
%invoke path follower and orientation controller such that the desired
%observation location is reached

cmd_movebot = ['echo "python orientationpathfollower.py ',num2str(opt_action(1)), ' ', num2str(opt_action(2)), ' ', num2str(opt_action(3)), '"', '|nc nucaddress'];
[status, cmdout] = system(cmd_movebot);
