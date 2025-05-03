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

load('matfiles\linear_model.mat')
load('matfiles\trim_op_fixed_v.mat')

% Sampling Time
ts  = 0.1;

% plant = LTI_model;
discrete_model = c2d(lin_model,ts);
plant = ss(discrete_model.A, discrete_model.B, discrete_model.C,discrete_model.D,ts);
% plant = ss(lin_model.A, lin_model.B, lin_model.C,lin_model.D);
plant = setmpcsignals(plant,'MV',[1 2],'MD',[3 4]);

%% Define tuning parameters

% Prediction horizon
Ph = 100; % N steps of prediction

% Control Horizon
Ch = 10; % N steps of control

% Control Weights
W = struct();

% Respectively - roll height
W.OutputVariables = [1 1];  % Weights for the 2 MOs

% Respectively - aileron and rear foil
W.ManipulatedVariables = [0.5 1];  % Weights for the 2 MVs
W.ManipulatedVariablesRate = [0 0];  % Weights for rate of change of the 2 MVs

%% Defining Constraints
% Manipulated variables properties
min_ailerons = -15;
max_ailerons = 15;
min_aileron_rate = -99; % min angle of attack rate
max_aileron_rate = 99; % max angle of attack rate

minAOA_rear = -3; 
maxAOA_rear = 15; 
min_rear_rate = -99; % min angle of attack rate
max_rear_rate = 99; % max angle of attack rate

% Operating point conditions

MV(1) = struct('Min', min_ailerons, 'Max', max_ailerons, 'RateMin', min_aileron_rate*ts, 'RateMax', max_aileron_rate*ts, 'ScaleFactor', 1);
MV(2) = struct('Min', minAOA_rear, 'Max', maxAOA_rear, 'RateMin', min_rear_rate*ts, 'RateMax', max_rear_rate*ts, 'ScaleFactor', 1);

% Measured Outputs properties
minTheta = -10;
maxTheta = 10;
minZ = -10;
maxZ = 150;

OV(1) = struct('Min', minTheta, 'Max', maxTheta, 'ScaleFactor', 1);
OV(2) = struct('Min', minZ, 'Max', maxZ, 'ScaleFactor', 1);

%% Create MPC object
mpcobj = mpc(plant, ts, Ph, Ch, W, MV, OV);

% Dont use built in Kalman Filter
setEstimator(mpcobj,"custom")

% Set IO atributes
setname(mpcobj,'input',1,'Ailerons')
setname(mpcobj,'input',2,'Rear Foil')
setname(mpcobj,'input',3,'RPMS')
setname(mpcobj,'input',4,'Rudder')

setname(mpcobj,'Output',1,'Roll')
setname(mpcobj,'Output',2,'Heave')

% Set Initial Condition
state = [
    initial_uspeed, initial_vspeed, initial_wspeed,...
    initial_P, initial_Q, initial_R,...
    initial_Pitch, initial_ROLL,...
    initial_Z
        ];

initial_MV = [act_ailerons,act_rear];
disturbances = [T, gg];

mpcobj.Model.Nominal.X = state;
mpcobj.Model.Nominal.U = [initial_MV,disturbances];
mpcobj.Model.Nominal.Y = [initial_ROLL,initial_Z];

controller_state = mpcstate(mpcobj);

% Otimization parameters (so in c++ code is the same)
% n = 4 x (n_c + n_v) // default
n_v = 2 * Ch; % Manipualted Variable * Control Horizon
n_c = 4 * Ph; % (Output Constraints) * Prediction Horizon
n = 4 * (n_v + n_c);

mpcobj.Optimizer.SolverOptions.MaxIterations = n;
mpcobj.Optimizer.SolverOptions.ConstraintTolerance = 1e-6; % default

% mpcDesigner(mpcobj)
% noise_model = mpcobj.getoutdist

save('controller.mat','mpcobj','controller_state','ts')