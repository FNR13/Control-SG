clear
%% Load Linearized Model

%Uncomment to linearize
%run('../../NonlinearBoatModel/linearize_boat');
load('MatFiles/linear_model.mat')

var = [1 2 3 4 5 7 9];

A = lin_model.A(var,var);
B = [lin_model.B(var,1) lin_model.B(var,2) lin_model.B(var,3)];
C = [0 0 0 0 1 0 0;
     0 0 0 0 0 0 1;
     0 0 0 1 0 0 0];
D = 0;

sys = ss(A, B, C, D);

%% LQI's algorithm

% New matrices to calculate LQI
% longitudinal

Ai=[A   , zeros(size(A,1),3);
    -C   ,        zeros(3,3)];
Bi=[ B; 
     zeros(3,3)];    
Ci=[C, zeros(3,3)];
Di=D;

%% Set weights manually 

% Heave and pitch
theta_range = 1;
thetadot_range = 2; 
u_range = 10;
w_range = 4;
z_range = 1;

rolldot_range = 10;
roll_range = 6;
foil_range = 8.5;

weights = [1 1 1 1 1 1000 1];

Qh = [weights(1)/u_range^2 weights(2)/w_range^2 weights(3)/thetadot_range^2 ...
       weights(4)/theta_range^2 weights(5)/z_range^2]; 

Ql = [weights(6)/rolldot_range^2 weights(7)/roll_range^2];

weights = [700 700 1000];
  
Rh = weights(1)/foil_range^2;

Rl = weights(2)/foil_range^2;

Rp = weights(3)/foil_range^2;

int_z = 100;
int_roll = 100;
int_pitch = 1;




%% Calculate gains
Q = diag([Qh Ql int_z int_roll int_pitch]);
R = diag([Rh Rl Rp]);

[Ki,~,~]=lqr(Ai,Bi,Q,R)

%Ki(:, 6) = [0.6; -0.6; 0];
%Ki(:, 7) = [4; -4; 0]

% save('controller.mat','Ki')



