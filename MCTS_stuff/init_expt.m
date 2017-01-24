%initialises all the relevant variables

clear
close all

%size of the map- actual dimensions depend on size of the Mars Lab and the
%required resolution
MapParameters.xsize = 220;
MapParameters.ysize = 760;

%MapParameters.xsize = 1000;
%MapParameters.ysize = 1500;

%resolution of the location and silica cells
%1 cell = 20x20 gridrobot_pos
MapParameters.l_rows = MapParameters.xsize/20;
MapParameters.l_cols = MapParameters.ysize/20;



%Domain Knowledge parameters- no uncertainty
%P(R|L)
% Done by CPTs -->         L
%                  |---------------|
%                  |   | 0 | 1 | 2 |
%                  |---|---|---|---|
%                  | 0 |
%                R | 1 |
%                  | 2 | 
%                  |---|-----------|

%R = 0 corresponds to no rock existed there
%R = 1 rock type 1 detected
%R = 2 rock type 2 detected
%theta_rl = [0.8 0.18 0.02; 0.6 0.38 0.02; 0.4 0.4 0.2];
theta_rl = [0.1 0.2 0.7; 0.2 0.6 0.2; 0.6 0.35 0.05];
DomainKnowledge.theta_rl = theta_rl';


%P(F|R)
% Done by CPTs -->         R
%                  |---------------|
%                  |   | 0 | 1 | 2 |
%                  |---|---|---|---|A 
%                  | 0 |
%                F | 1 |
%                  | 2 | 
%                  |---|-----------|

%F = 0 corresponds to no observation detected
%F = 1 feature 1 not present but there was a rock
%F = 2 feature 2 present given there is a rock
%DomainKnowledge.theta_fr_1 = [1 0 0; 0 0.9 0.1; 0 0.5 0.5]';  
%DomainKnowledge.theta_fr_2 = [1 0 0; 0 0.4 0.6 ; 0 0.2 0.8]'; 
%DomainKnowledge.theta_fr_3 = [1 0 0; 0 0.85 0.15; 0 0.75 0.25]';

% DomainKnowledge.theta_fr_1 = [0.8 0.15 0.05; 0.1 0.8 0.1; 0.1 0.5 0.4]';  
% DomainKnowledge.theta_fr_2 = [0.7 0.2 0.1; 0.1 0.4 0.5 ; 0.05 0.15 0.8]'; 
% DomainKnowledge.theta_fr_3 = [0.75 0.05 0.2; 0.3 0.65 0.05; 0.1 0.75 0.15]';
DomainKnowledge.theta_fr_1 = [0.5 0.4 0.1;0.2 0.6 0.2;0.05 0.15 0.8]'; 

DomainKnowledge.theta_fr_2 = [0.5 0.4 0.1;0.1 0.2 0.7;0.6 0.3 0.1]';  

DomainKnowledge.theta_fr_3 = [0.6 0.3 0.1;0.2 0.4 0.4;0.7 0.2 0.1]';


%P(S|L)
% Done by CPTs -->         L
%                  |---------------|
%                  |   | 0 | 1 | 2 |
%                  |---|---|---|---|
%                  | 0 |
%                S | 1 |
%                  | 2 | 
%                  |---|-----------|

%S = 0 no silica present
%S = 1 silica present in mild quantities
%S = 2 silica present in high quantities
%DomainKnowledge.theta_sl = [0.95 0.04 0.01; 0.6 0.35 0.05; 0.5 0.25 0.25]';
DomainKnowledge.theta_sl = [0.1 0.8 0.1;0.05 0.25 0.7;0.75 0.2 0.05]';

%initialise beliefs
BeliefMaps.Location = cell(MapParameters.l_rows,MapParameters.l_cols);
BeliefMaps.Silica   = cell(MapParameters.l_rows,MapParameters.l_cols);

BeliefMaps.Rock     = cell(MapParameters.xsize,MapParameters.ysize);
BeliefMaps.F1       = cell(MapParameters.xsize,MapParameters.ysize);
BeliefMaps.F2       = cell(MapParameters.xsize,MapParameters.ysize);
BeliefMaps.F3       = cell(MapParameters.xsize,MapParameters.ysize);


priorLoc  = [1/3 1/3 1/3];

%no uncertainty in parameters..
priorRock = DomainKnowledge.theta_rl*priorLoc';
prior_f1  = DomainKnowledge.theta_fr_1*priorRock;
prior_f2  = DomainKnowledge.theta_fr_2*priorRock;
prior_f3  = DomainKnowledge.theta_fr_3*priorRock;
prior_S   = DomainKnowledge.theta_sl*priorLoc';
    
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


%robot initial position and orientation
robot.xpos = 200;
robot.ypos = 300;
robot.orientation = 0;

%sensor parameters
sensor.x_trans = 0;
sensor.y_trans = 25;
%sensor.FOV.width = 42;
%sensor.FOV.height = 24;
sensor.FOV.width = 35;
sensor.FOV.height = 21;
sensor.FOV.coords = getsensorFOV(sensor);

%noise parameters
% Done by CPTs -->         F
%                  |---------------|
%                  |   | 0 | 1 | 2 |
%                  |---|---|---|---|
%                  | 0 |
%                Z | 1 |
%                  | 2 | 
%                  |---|-----------|

%F = 0 corresponds to no rock existed there
%F = 1 desired feature is not there
%F = 2 feature is there

%arbitary
%sensor.noise_f1 = [0.9 0.05 0.05; 0.1 0.8 0.1; 0.05 0.25 0.7];

sensor.noise_f1 = [0.9 0.1 0.05; 0.05 0.7 0.15; 0.05 0.2 0.8];
%sensor.noise_f1 = [0.9 0.1;0.1 0.9];

%initialise visibility map
robot.visibility = zeros(MapParameters.xsize, MapParameters.ysize);

%initialise previous obs map
robot.obshistory = cell(MapParameters.xsize,MapParameters.ysize);

%initialise old message storage (for computational efficiency)
robot.oldmsgs = cell(MapParameters.xsize,MapParameters.ysize);

%initialise robot mode (remote=0 or local sense=1)
robot.mode = 0;

%initialise action space
num_orientations = 4;
action_space = zeros(MapParameters.l_rows, MapParameters.l_cols, num_orientations);

%Mars Yard origin location [x y]
yard_origin = [4.7741,-3.1006];

load feature_map_1.mat;
load feature_map_2.mat;
load feature_map_3.mat;
load rock_map.mat;
load location_map.mat;
load silica_map.mat;
load location_ground_truth.mat;

load traversability_map.mat;

%this map is in terms of total grid size- not in terms of locations

MapParameters.traversabilitymap = traversability_map;
