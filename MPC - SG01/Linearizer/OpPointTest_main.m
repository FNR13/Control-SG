%clc
%clear

% Act_Ailerons = 0;
% Act_Rear = 4;
% Rudder = 0;
% Uinf = 10.28;
% Pitch0 = -3.558;
% z_cm = 50;

act_ailerons = 0;
act_rear = 4;
T = 2679.2979205956;
Rudder = 0;

initial_uspeed = cos(deg2rad(-3.558))*10.275;
initial_vspeed = 0;
initial_wspeed = sin(deg2rad(-3.558))*10.275;

initial_P = 0;
initial_Q = 0;
initial_R = 0;

% initial_Pitch = 0;
initial_Pitch = -3.558;
initial_ROLL = 0;
initial_YAW = 0;

initial_Z = 100;

 sim("OpPointTest")









