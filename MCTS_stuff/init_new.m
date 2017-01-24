%initialises all the relevant variables

clear
close all

%number of different location regions
num_x = 4;
num_y = 4;

%number of location per grid
grid_size = 8;

% %number of different location regions
% num_x = 5;
% num_y = 5;
% 
% %number of location per grid
% grid_size = 5;


%resolution of the location and silica cells
%1 cell = 20x20 grid
MapParameters.l_rows = num_x*grid_size;
MapParameters.l_cols = num_y*grid_size;

%size of the rock and feature maps- ratio of 20
MapParameters.xsize = MapParameters.l_rows*20;
MapParameters.ysize = MapParameters.l_cols*20;

knowledge_accuracy = 1;

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

%simulation setting
%theta_rl = [0.8 0.18 0.02; 0.6 0.38 0.02; 0.4 0.4 0.2];

%JFR setting
theta_rl = [0.75,0.05,0.2; 0.05,0.8,0.15; 0.15,0.05,0.8]; 
DomainKnowledge_true.theta_rl = theta_rl';

theta_rl = knowledge_accuracy*theta_rl + (1-knowledge_accuracy)*(1/3)*ones(3,3);


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

%simulation setting
% DomainKnowledge.theta_fr_1 = [0.8 0.15 0.05; 0.1 0.8 0.1; 0.1 0.5 0.4]';  
% DomainKnowledge.theta_fr_2 = [0.7 0.2 0.1; 0.1 0.4 0.5 ; 0.05 0.15 0.8]'; 
% DomainKnowledge.theta_fr_3 = [0.75 0.05 0.2; 0.3 0.65 0.05; 0.1 0.75 0.15]';
% 
% DomainKnowledge_true.theta_fr_1 = [0.8 0.15 0.05; 0.1 0.8 0.1; 0.1 0.5 0.4]';  
% DomainKnowledge_true.theta_fr_2 = [0.7 0.2 0.1; 0.1 0.4 0.5 ; 0.05 0.15 0.8]'; 
% DomainKnowledge_true.theta_fr_3 = [0.75 0.05 0.2; 0.3 0.65 0.05; 0.1 0.75 0.15]';

%JFR setting
DomainKnowledge.theta_fr_1 = [0.05,0.3,0.65; 0.1,0.5,0.4; 0.7,0.29,0.01]';
DomainKnowledge.theta_fr_2 = [0.5,0.45,0.05; 0.01,0.19,0.8; 0.4,0.55,0.05]';  
DomainKnowledge.theta_fr_3 = [0.4,0.2,0.4; 0.15,0.75,0.1; 0.3,0.35,0.35]';

DomainKnowledge_true.theta_fr_1 = [0.05,0.3,0.65; 0.1,0.5,0.4; 0.7,0.29,0.01]';
DomainKnowledge_true.theta_fr_2 = [0.5,0.45,0.05; 0.01,0.19,0.8; 0.4,0.55,0.05]';  
DomainKnowledge_true.theta_fr_3 = [0.4,0.2,0.4; 0.15,0.75,0.1; 0.3,0.35,0.35]';



%more accurate knowledge
% DomainKnowledge.theta_fr_1 = [0.8 0.15 0.05; 0.15 0.8 0.05; 0.05 0.15 0.8]';  
% DomainKnowledge.theta_fr_2 = [0.8 0.15 0.05; 0.15 0.8 0.05; 0.05 0.15 0.8]';  
% DomainKnowledge.theta_fr_3 = [0.8 0.15 0.05; 0.15 0.8 0.05; 0.05 0.15 0.8]';  
% 
% DomainKnowledge_true.theta_fr_1 = [0.8 0.15 0.05; 0.15 0.8 0.05; 0.05 0.15 0.8]';  
% DomainKnowledge_true.theta_fr_2 = [0.8 0.15 0.05; 0.15 0.8 0.05; 0.05 0.15 0.8]';  
% DomainKnowledge_true.theta_fr_3 = [0.8 0.15 0.05; 0.15 0.8 0.05; 0.05 0.15 0.8]'; 

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
%DomainKnowledge.theta_sl = [0.8 0.15 0.05; 0.3 0.65 0.05; 0.2 0.3 0.5]';

%DomainKnowledge.theta_sl = [0.8 0.15 0.05; 0.3 0.65 0.05; 0.05 0.3 0.65]';
%theta_sl = [0.95 0.04 0.01; 0.6 0.35 0.05; 0.5 0.25 0.25]';

%theta_sl = [0.8 0.05 0.15; 0.15 0.8 0.05; 0.05 0.15 0.8]';

%simulation setting
%theta_sl = [0.9 0.05 0.05; 0.05 0.9 0.05; 0.05 0.05 0.9]';

%theta_sl = [0.8 0.1 0.1; 0.1 0.8 0.1; 0.1 0.1 0.8]';
theta_sl = [0.9 0.05 0.05; 0.05 0.9 0.05; 0.05 0.05 0.9]';

DomainKnowledge_true.theta_sl = theta_sl;

theta_sl = knowledge_accuracy*theta_sl + (1-knowledge_accuracy)*(1/3)*ones(3,3);
DomainKnowledge.theta_sl = theta_sl;

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
sensor.y_trans = 50;
sensor.FOV.width = 50;
sensor.FOV.height = 40;
sensor.FOV.coords = getsensorFOV(sensor);

%new sensor parameters
% sensor.x_trans = 0;
% sensor.y_trans = 40;
% sensor.FOV.width = 40;
% sensor.FOV.height = 32;
% sensor.FOV.coords = getsensorFOV(sensor);

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

%sensor.noise_f1 = [0.9 0.1 0.05; 0.05 0.7 0.15; 0.05 0.2 0.8];
sensor.noise_f1 = [0.9 0.05 0.05; 0.05 0.9 0.05; 0.05 0.05 0.9];
%sensor.noise_f1 = [0.9 0.1;0.1 0.9];

%initialise visibility map
robot.visibility = zeros(MapParameters.xsize, MapParameters.ysize);
robot.visibility_silica = zeros(MapParameters.l_rows, MapParameters.l_cols);

%initialise previous obs map
robot.obshistory = cell(MapParameters.xsize,MapParameters.ysize);

%initialise old message storage (for computational efficiency)
robot.oldmsgs = cell(MapParameters.xsize,MapParameters.ysize);

%initialise robot mode (remote=0 or local sense=1)
robot.mode = 0;

%initialise action space
num_orientations = 4;
action_space = zeros(MapParameters.l_rows, MapParameters.l_cols, num_orientations);
% 
% load feature_map_1.mat;
% load feature_map_2.mat;
% load feature_map_3.mat;
% load rock_map.mat;
% load location_map.mat;
% load silica_map.mat;
% load location_ground_truth.mat;

