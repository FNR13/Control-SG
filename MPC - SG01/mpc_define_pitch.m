clc
clear
%% Defining mpc object

% mpcobj = mpc(plant,ts,P,M,W,MV,OV,DV);
%
% Plant - LTI model
%
% Ts sets the sampling time. (seconds)
%
% P sets the PredictionHorizon property. (Scalar)
% 
% M sets the ControlHorizon property. (Scalar)
% 
% W sets the Weights property for the control actions, control rates and reference following (strut)
% 
% MV sets the limits of the manipulated variables (min and max value rates). (strut)
% 
% OV sets the OutputVariables property.
% 
% DV sets the DisturbanceVariables property.


%% Load Linearized Model

load('MatFiles\linear_model_p.mat')
load('MatFiles\trim_op_fixed_v.mat')

% plant = LTI_model;
plant = ss(lin_model_p.A, lin_model_p.B, lin_model_p.C,lin_model_p.D);
plant = setmpcsignals(plant,'MV',[1 2],'MD',[3 4]);

%% Define tuning parameters

% Sampling Time
ts  = 0.020;

% Prediction horizon
P = 100; % N steps of prediction

% Control Horizon
M = 10; % N steps of control


% Control Weights
W = struct();

% Respectively - aileron and rear foil
W.ManipulatedVariables = [1 1];  % Weights for the 2 MVs
W.ManipulatedVariablesRate = [1 1];  % Weights for rate of change of the 2 MVs

% Respectively - pitch roll height
W.OutputVariables = [100 1 1];  % Weights for the 2 MOs

%% Defining Constraints
% Manipulated variables properties
min_ailerons = -15;
max_ailerons = 15;
min_aileron_rate = -inf; % min angle of attack rate
max_aileron_rate = inf; % max angle of attack rate

minAOA_rear = -3; 
maxAOA_rear = 15; 
min_rear_rate = -inf; % min angle of attack rate
max_rear_rate = inf; % max angle of attack rate

MV(1) = struct('Min', min_ailerons, 'Max', max_ailerons, 'RateMin', min_aileron_rate, 'RateMax', max_aileron_rate, 'ScaleFactor', 1);
MV(2) = struct('Min', minAOA_rear, 'Max', maxAOA_rear, 'RateMin', min_rear_rate, 'RateMax', max_rear_rate, 'ScaleFactor', 1);

% Measured Outputs properties
minPitch = -10;
maxPitch = 10;
minTheta = -5;
maxTheta = 5;
minZ = -200;
maxZ = 0;

OV(1) = struct('Min', minPitch, 'Max', maxPitch, 'ScaleFactor', 1);
OV(2) = struct('Min', minTheta, 'Max', maxTheta, 'ScaleFactor', 1);
OV(3) = struct('Min', minZ, 'Max', maxZ, 'ScaleFactor', 1);

%% Create MPC object
mpcobj_p = mpc(plant, ts, P, M, W, MV, OV);

% Dont use built in Kalman Filter
setEstimator(mpcobj_p,"custom")

% Set IO atributes
setname(mpcobj_p,'input',1,'Ailerons')
setname(mpcobj_p,'input',2,'Rear Foil')
setname(mpcobj_p,'input',3,'RPMS')
setname(mpcobj_p,'input',4,'Rudder')

setname(mpcobj_p,'Output',1,'Roll')
setname(mpcobj_p,'Output',2,'Heave')

% Set Initial Condition

state = [initial_uspeed,initial_vspeed,initial_wspeed,...
         initial_P, initial_P,initial_P,...
         initial_Pitch,initial_ROLL,initial_Z];

initial_MV = [act_ailerons,act_rear];
disturbances = [T, gg];

mpcobj_p.Model.Nominal.X = state;
mpcobj_p.Model.Nominal.U = [initial_MV,disturbances];
mpcobj_p.Model.Nominal.Y = [initial_Pitch,initial_ROLL,initial_Z];

controller_state_p = mpcstate(mpcobj_p);
% mpcDesigner(mpcobj)

save('controller_p.mat','mpcobj_p','controller_state_p')