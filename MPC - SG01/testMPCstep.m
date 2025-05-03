clear 
clc

load('controller.mat')
r = [5 60];
y = [0 50];
x = mpcstate(mpcobj);

u = mpcmove(mpcobj,x,y,r) - [0; 4]

