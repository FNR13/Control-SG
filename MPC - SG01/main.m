clc
clear

% prompt = {['Enter the speed at which to start:' newline ...
%            'Trim can not be achieved below 5.5 and above 9']};
%trim_boat(str2double(inputdlg(prompt)),-0.4);

run ("mpc_define.m")
load('controller.mat')

% load('controller_p.mat')
load('MatFiles/trim_op_fixed_v.mat');
load('MatFiles/linear_model.mat');


ultrasonic_delay = 0.3;
n = round(ultrasonic_delay/ts);

% sim("main_SG_MPC")









