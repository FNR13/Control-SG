clc
clear
% load('Matfiles/linear_model.mat','initial_uspeed')
% prompt = {['Do you wish to linearize at a different speed?' newline ...
%            'Can not trim for speeds greater than 9 m/s (propeller)' ...
%            newline 'Current speed: ' num2str(initial_uspeed)]};
% 
% answer = str2double(inputdlg(prompt));
% if any(isnan(answer)) || isempty(answer) 
%    fprintf('Did not linearize\n')
%    return
% end

% trim_boat_SG(answer, -0.4);

% before linearizing the inputs and outputs must be chosen on the model
% get operating point before linearizing

load('trim_op_fixed_v.mat')

% close and open model to call callbacks
model = 'SG_model';
close_system(model,0)
load_system(model)

%get the inputs of the model
io = getlinio(model);

%now specify the ouputs of the linearized model
% io(size(io,1)+1) = linio('SG_model/aUSPEED',1,'output');
io(size(io,1)+1) = linio('SG_model/hRoll',1,'output');
io(size(io,1)+1) = linio('SG_model/jZ',1,'output');

%linearize
lin_model = linearize(model,io,op);
A = lin_model.A;
B = lin_model.B;
C = lin_model.C;
D = lin_model.D;

save('linear_model.mat','lin_model','initial_uspeed')

% Controlability: rank must be equal to dim of model
Co = ctrb(lin_model);
Co_rank = rank(Co);

if Co_rank < size(lin_model.A,1)
   error("model is not controllable");
end

% simplify roll model to 2 states, only roll and rolldot matter
% and consider only 1 output (dif mode)
lateral_model = [2 4 6 8];
Al = lin_model.A(lateral_model,lateral_model);
Bl = [lin_model.B(lateral_model,1) lin_model.B(lateral_model,4)];
Cl = [0 0 0 1];
Dl = 0;

lin_model_l = ss(Al,Bl,Cl,Dl);

% Controlability: rank must be equal to dim of model
Co = ctrb(lin_model_l);
Co_rank = rank(Co);
if Co_rank < size(lin_model_l.A,1)
    error("lateral model is not controllable");
end









