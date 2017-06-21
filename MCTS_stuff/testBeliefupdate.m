%testing if belief space update works correctly

tot_num = 50;
num_actions = 75;

ent_data = zeros(num_actions, tot_num);
cost_data = zeros(num_actions, tot_num);
recog_scoredata = zeros(num_actions, tot_num);
unexplored_celldata = zeros(num_actions, tot_num);

cost_remote = 1;
cost_local = 8;

tot_budget = 75;

robot.xpos = 20; %100 cell padding to prevent rovers starting from the edge
robot.ypos = 20;
robot.orientation = 0;

robot.visibility = zeros(MapParameters.xsize, MapParameters.ysize);
robot.visibility_silica = zeros(MapParameters.l_rows, MapParameters.l_cols);

%initialise previous obs map
robot.obshistory = cell(MapParameters.xsize,MapParameters.ysize);

%initialise old message storage (for computational efficiency)
robot.oldmsgs = cell(MapParameters.xsize,MapParameters.ysize);

%initialise robot mode (remote=0 or local sense=1)
robot.mode = 0;

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

if robot.mode == 0, %remote sense is to be made
    %get the visible cells
    [visible_cells, new_FOV] = getVisibleCells([robot.xpos, robot.ypos], robot.orientation, sensor.FOV.coords, MapParameters);
    
    %get observation vector based on the map
    [ obs_vect ] = generate_obs(feature_map_1,feature_map_2, feature_map_3, visible_cells, sensor);
    
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
    obs_vect(1) = silica_map(visible_cells(1), visible_cells(2));
    obs_vect(4) = 1;
    obs_vect(5) = visible_cells(1);
    obs_vect(6) = visible_cells(2);
end

%update belief space based on the observations
[BeliefMaps, robot, infoGain_actual] = updateBeliefSpace(BeliefMaps, obs_vect, robot, sensor, DomainKnowledge, MapParameters);

disp('Actual information gained: ')
disp(infoGain_actual)
actualinfogain_vect(iterate) = infoGain_actual;


