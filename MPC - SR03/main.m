clc
clear
prompt = {['Enter the speed at which to start:' newline ...
           'Trim can not be achieved below 5.5 and above 9']};
%trim_boat(str2double(inputdlg(prompt)),-0.4);

%run('../Controllers/LQI_rear_fixed/lqi')
%load('../Controllers/LQI_rear_fixed/controller.mat')
%run('../Controllers/LQR/clqr')
%load('../Controllers/LQR/controller.mat')
% run('lqi')
load('controller.mat')
load('MatFiles/trim_op_fixed_v.mat');
load('MatFiles/motor_efficiency.mat');
load('MatFiles/FilterCoef.mat');
%Kil = [0.1 0.8 -0.5]; %%better gains found experimentally











