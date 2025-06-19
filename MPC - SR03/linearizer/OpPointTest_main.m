clc
clear
initial_uspeed = 7;
initial_Z = -0.4;

% Act_Ailerons = 0;
% Act_Rear = 4;
% Rudder = 0;
% Uinf = 10.28;
% Pitch0 = -3.558;
% z_cm = 50;

FF_L = 1.6106; % 1.6106
FF_R = 1.6106; % 1.6106
rear_alfas = 1.9060; % 1.9060

T = 4770.6;
Rudder = 0;

initial_uspeed = 7; % 10.28
initial_vspeed = 0;
initial_wspeed = 0;

initial_P = 0;
initial_Q = 0;
initial_R = 0;

initial_Pitch = 0;
% initial_Pitch = -3.558;
initial_ROLL = 0;
initial_YAW = 0;

initial_Z = -0.40;

 sim("OpPointTest")









