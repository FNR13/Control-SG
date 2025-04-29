clc
clear


load('MatFiles/trim_op_fixed_v.mat');

kp_roll = 4.3;
ki_roll = 1.4;
kd_roll = 0.7;
kaw_roll = 0.06;

kp_height = 0.43;
ki_height = 0.14;
kd_height = 0.07;
kaw_height = 0.0001;


ultrasonic_delay = 0.3;
Ts = 0.1;
n = round(ultrasonic_delay/Ts);

min_ailerons = -15;
max_ailerons = 15;
min_aileron_rate = -5; % min angle of attack rate
max_aileron_rate = 5; % max angle of attack rate

min_rear = -3; 
max_rear = 15; 
min_rear_rate = -2.5; % min angle of attack rate
max_rear_rate = 2.5; % max angle of attack rate

trim_ailerons = 0;
trim_rear = 4;

sim("main_SG_PID",60)











