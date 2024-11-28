clc 
clear
%% Search for a specified operating point for the model - SG_model.

%ATTENTION BEFORE RUNNING
%open the 'SG_model' and set all 4 inputs as inputs with the manual
%switches

% too have free rear foil put -10 as input;
% too have free thrust put 0 as input;

% function [] = trim_boat_SG(initial_uspeed,initial_Z)

initial_uspeed = 10.285;
initial_Z = -50;
initial_Pitch = -1.31;

foilborn = 1;
Rpms_motor = 4650;
rear = 2.35;
aileron = 0;

% wait bar
f = uifigure('Name','Wait','Position',[200 200 425 275]);
d = uiprogressdlg(f,'Title','Searching equilibrium point','Indeterminate','on');



%% Specify the model name
model = 'SG_model';

%set max and min values for trim
min_ailerons = -15;
max_ailerons = 15;
minAOA_rear = -3; 
maxAOA_rear = 15; 
minRpms_motor = 0;
maxRpms_motor = 10000; %max rpms
minGG = -15; % Rudder angle
maxGG = 15;

minU = 0;
maxU = 20;
minV = -5;
maxV = 5;
minW = -5;
maxW = 5;

minPITCH = -5;
maxPITCH = 5;
minYAW = -50000;
maxYAW = 50000;

minZ = -1000;
maxZ = 1000;

% Rpms_motor = 1000*initial_uspeed;


% if initial_Z == -0.2
%     foilborn = 0;
%     angulo_frente = -3;
%     angulo_tras = -3;
% else
%     foilborn = 1;
%     angulo_frente = 7;
%     angulo_tras = 7;
% end
% if initial_uspeed >= 7 && initial_Z < -0.3
%     angulo_frente = 1;
%     angulo_tras =1;
% end
% if initial_uspeed < 0.6
%     Rpms_motor = 2300*initial_uspeed;
% end


%% Set initial conditions
x0 = [initial_uspeed; 0;0;0;0;0;initial_Pitch;0;0; initial_Z];
u0 = [aileron; rear; Rpms_motor;0];
y0 = [0 0 0 0 0 0 0 0];
% create constants for simulink model
assignin('base','initial_uspeed',x0(1));
assignin('base','initial_wspeed',x0(2));
assignin('base','initial_vspeed',x0(3));

assignin('base','initial_P',x0(4));
assignin('base','initial_Q',x0(5));
assignin('base','initial_R',x0(6));

assignin('base','initial_Pitch',x0(7));
assignin('base','initial_ROLL',x0(8));
assignin('base','initial_YAW',x0(9));

assignin('base','initial_Z',x0(10));

%% Set the constraints on the states in the model.
% - The defaults for all states are Known = false, SteadyState = true,
%   Min = -Inf, Max = Inf, dxMin = -Inf, and dxMax = Inf.


%% Create the operating point specification object.
opspec = operspec(model);

%% Set the constraints on the states in the model.
% - The defaults for all states are Known = false, SteadyState = true,
%   Min = -Inf, Max = Inf, dxMin = -Inf, and dxMax = Inf.

for i=1:length(opspec.States)
    opspec.States(i).x = x0(i);
end

for i=1:length(opspec.Outputs)
    opspec.Outputs(i).y = y0(i);
end


for i=1:(length(opspec.States))
    opspec.States(i).Known = true;
end

% Longitudinal
opspec.States(1).SteadyState = 1;
opspec.States(3).SteadyState = 1;
opspec.States(5).SteadyState = 1;
opspec.States(7).SteadyState = 1;
opspec.States(10).SteadyState = 0;

% Lateral
opspec.States(2).SteadyState = 0;
opspec.States(4).SteadyState = 0;
opspec.States(6).SteadyState = 0;
opspec.States(8).SteadyState = 0;
opspec.States(9).SteadyState = 1;

%% Set the constraints on the inputs in the model.
% - The defaults for all inputs are Known = false, Min = -Inf, and
% Max = Inf.

for i=1:length(opspec.inputs)
    opspec.Inputs(i).u = u0(i);
end

opspec.Inputs(1).known = 1;
opspec.Inputs(2).known = 0;
opspec.Inputs(3).known = 0;
opspec.Inputs(4).known = 1;

%% set the max and mins

% Input (1) - SG_model/ailerons
opspec.Inputs(1).Min = min_ailerons;
opspec.Inputs(1).Max = max_ailerons;
% Input (2) - SG_model/act rear
opspec.Inputs(2).Min = minAOA_rear;
opspec.Inputs(2).Max = maxAOA_rear;
% Input (3) - SG_model/thrust
opspec.Inputs(3).Min = minRpms_motor;
opspec.Inputs(3).Max = maxRpms_motor;
% Input (4) - SG_model/rudder
opspec.Inputs(4).Min = minGG;
opspec.Inputs(4).Max = maxGG;

% States (1) - SG_model/uspeed
opspec.States(1).Min = minU;
opspec.States(1).Max = maxU;

% States (2) - SG_model/v
opspec.States(2).Min = minV;
opspec.States(2).Max = maxV;

% States (3) - SG_model/wspeed
opspec.States(3).Min = minW;
opspec.States(3).Max = maxW;

% States (7) - SG_model/pitch
opspec.States(7).Min = minPITCH;
opspec.States(7).Max = maxPITCH;

% States (9) - SG_model/yaw
opspec.States(9).Min = minYAW;
opspec.States(9).Max = maxYAW;

% States (10) - SG_model/z
opspec.States(10).Min = minZ;
opspec.States(10).Max = maxZ;


%% Set the constraints on the outputs in the model.
% - The defaults for all outputs are Known = false, Min = -Inf, and
% Max = Inf.

%% Create the search options
opt = findopOptions;

opt.DisplayReport = 'iter';
opt.OptimizerType = 'graddescent-elim';


%% Perform the operating point search
    
    [op,opreport] = findop(model,opspec,opt)
    

    act_ailerons = op.inputs(1).u;
    act_rear = op.inputs(2).u;
    T = op.inputs(3).u;
    gg = op.inputs(4).u;

    initial_uspeed = op.states(1).x;
    initial_vspeed = op.states(2).x;
    initial_wspeed = op.states(3).x;

    initial_P = op.states(4).x;
    initial_Q = op.states(5).x;
    initial_R = op.states(6).x;

    initial_Pitch = op.states(7).x;
    initial_ROLL = op.states(8).x;
    initial_YAW = op.states(9).x;

    initial_Z = op.states(10).x;

    save('trim_op_fixed_v.mat','op','opreport', ...
        'act_ailerons','act_rear','T','gg', ...
        'initial_uspeed','initial_vspeed','initial_wspeed', ...
        'initial_P','initial_R','initial_Q', ...
        'initial_Pitch','initial_ROLL','initial_YAW', ...
        'initial_Z')

% end
