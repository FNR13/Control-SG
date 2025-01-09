clc
clear


act_rear = 2;
T = 4652;

initial_uspeed = 10.285;
initial_vspeed = 0;
initial_wspeed = 0;

initial_P = 0;
initial_Q = 0;
initial_R = 0;

% initial_Pitch = 0;
initial_Pitch = -1.31;
initial_ROLL = 0;
initial_YAW = 0;

initial_Z = -50;

sim("OpPointTest.slx")









