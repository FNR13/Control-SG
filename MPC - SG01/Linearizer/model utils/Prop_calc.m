clc
clear

% Just paste output in dynamic model file
% Read the table from the file
dados_prop = readtable('Props', 'Delimiter', ';');

% Extract the numerical data from the table
Js = table2array(dados_prop(2:end, 1));   % Js values
Kqs = table2array(dados_prop(2:end, 2));  % Kqs values
KTs = table2array(dados_prop(2:end, 3));  % KTs values
Effs = table2array(dados_prop(2:end, 4)); % Efficiency values

% Fit a ninth-degree polynomial model to Kq and KT, and a second-degree polynomial to Efficiency
Kq = fit(Js, Kqs, 'poly9');
KT = fit(Js, KTs, 'poly9');
Eff = fit(Js, Effs, 'poly9');

%% Coefficients for KT
fprintf('%% Coefficients for KT \n')

fprintf('KT_p1 = %.3f;\n',KT.p1)
fprintf('KT_p2 = %.3f;\n',KT.p2)
fprintf('KT_p3 = %.3f;\n',KT.p3)
fprintf('KT_p4 = %.3f;\n',KT.p4)
fprintf('KT_p5 = %.3f;\n',KT.p5)
fprintf('KT_p6 = %.3f;\n',KT.p6)
fprintf('KT_p7 = %.3f;\n',KT.p7)
fprintf('KT_p8 = %.3f;\n',KT.p8)
fprintf('KT_p9 = %.3f;\n',KT.p9)
fprintf('KT_p10 = %.3f;\n',KT.p10)
fprintf('\n')

fprintf('%% KT function \n')
fprintf('KT = @(x) KT_p1*x.^9 + KT_p2*x.^8 + KT_p3*x.^7 + KT_p4*x.^6 + KT_p5*x.^5 + KT_p6*x.^4 + KT_p7*x.^3 + KT_p8*x.^2 + KT_p9*x + KT_p10;\n')
fprintf('\n \n')


%% Coefficients for Kq
fprintf('%% Coefficients for kq \n')

fprintf('Kq_p1 = %.3f;\n',Kq.p1)
fprintf('Kq_p2 = %.3f;\n',Kq.p2)
fprintf('Kq_p3 = %.3f;\n',Kq.p3)
fprintf('Kq_p4 = %.3f;\n',Kq.p4)
fprintf('Kq_p5 = %.3f;\n',Kq.p5)
fprintf('Kq_p6 = %.3f;\n',Kq.p6)
fprintf('Kq_p7 = %.3f;\n',Kq.p7)
fprintf('Kq_p8 = %.3f;\n',Kq.p8)
fprintf('Kq_p9 = %.3f;\n',Kq.p9)
fprintf('Kq_p10 = %.3f;\n',Kq.p10)
fprintf('\n')

fprintf('%% Kq function \n')
fprintf('Kq = @(x) Kq_p1*x.^9 + Kq_p2*x.^8 + Kq_p3*x.^7 + Kq_p4*x.^6 + Kq_p5*x.^5 + Kq_p6*x.^4 + Kq_p7*x.^3 + Kq_p8*x.^2 + Kq_p9*x + Kq_p10;\n')
fprintf('\n \n')

%% Coefficients for Eff

fprintf('Eff_p1 = %.3f;\n',Eff.p1)
fprintf('Eff_p2 = %.3f;\n',Eff.p2)
fprintf('Eff_p3 = %.3f;\n',Eff.p3)
fprintf('Eff_p4 = %.3f;\n',Eff.p4)
fprintf('Eff_p5 = %.3f;\n',Eff.p5)
fprintf('Eff_p6 = %.3f;\n',Eff.p6)
fprintf('Eff_p7 = %.3f;\n',Eff.p7)
fprintf('Eff_p8 = %.3f;\n',Eff.p8)
fprintf('Eff_p9 = %.3f;\n',Eff.p9)
fprintf('Eff_p10 = %.3f;\n',Eff.p10)

fprintf('%% Eff function \n')
fprintf('Eff = @(x) Eff_p1*x.^9 + Eff_p2*x.^8 + Eff_p3*x.^7 + Eff_p4*x.^6 + Eff_p5*x.^5 + Eff_p6*x.^4 + Eff_p7*x.^3 + Eff_p8*x.^2 + Eff_p9*x + Eff_p10;\n')
fprintf('\n \n')
