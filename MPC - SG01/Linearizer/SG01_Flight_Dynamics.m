function [u_dot,v_dot,w_dot,p_dot,q_dot,r_dot,Pitch_dot,Roll_dot,Yaw_dot,Z_dot,Torque,Rpms_motor] = SG01_Flight_Dynamics(Act_Ailerons,Act_Rear,Rpms_motor,Rudder,u,v,w,p,q,r,Pitch,Roll,Yaw,z_cm)

% Modelo Dinâmico do SG01, recebendo o estado do sistema ele retorna as
% derivadas das velocidades da "embarcação"

%%% CONSTANTES %%%

g = 9.81; % Gravidade (m/s^2)
rho= 1026.00; % Densidade água salgada (kg/m^3)
mu= 1.14E-3; % Coef. viscosidade dinâmica água salgada (Pa*s)
m = 2000; % Massa da "embarcação" (Kg)
Ixx = 294.2;
Iyy = 1.788e+04; % Momentos de Inércia
Izz = 1.759e+04;
Ixz = 404.7; % Produto de Inércia relevante
b = 2.400; % Span da Asa
c = 0.30988; % Corda média da Asa
S = 0.68757769; % Área da Asa
S_t = 0.20158673; % Área Elevator

struts = 1;
wponto = 0;

if struts == 1 % struts estendidas, distancia submersa de strut
    z_struts = 1360 - z_cm/10; % estimativa muito rough
else
    z_struts = 0; % struts não estendidas INCLUIR AQUI TAMBÉM DIFERENTES DERIVADAS NO FUTURO
end


%%% DISTÂNCIAS %%% Esta secção deixou de ser necessária, acredito eu
x_cg_asas = 0.771; %centroide assumindo asa de tras semi eliptica e frente eliptica num referencial com origem a 20cm do leading edge da asa apontado para a cauda
X_CG_ASASRUDDERSTRUTS = (x_cg_asas*(S_t+S)-100*z_struts*2.825)/(S_t+S+100*z_struts); %X do CG das asa + rudder + struts no referencial com origem no meio da asa da frente CUIDADO COM O Z_STRUTS, que esta em mm
Z_CG_ASASRUDDERSTRUTS = (3*0.5*z_struts*100*z_struts)/(S_t+S+100*z_struts);
dist_CGSG_CGASAS_X = 3.715+X_CG_ASASRUDDERSTRUTS-3.964;
dist_CGSG_CGASAS_Y = 0; %idealmente
dist_CGSG_CGASAS_Z = z_struts + 1;

%%% Derivadas de estabilidade e controlo %%%
V0 = 10.285;
u0 = cos(deg2rad(-1.31))*V0;
w0 = sin(deg2rad(-1.31))*V0;
deltau = u - u0;
deltaw = w - w0;
deltaAct_Rear = 2.35 - Act_Rear;
teta0 = -1.31;
Cw0 = (m*g*cos(teta0))/(0.5*rho*(u0^2)*S);
Cd1 = 0.018;

CXu = -0.03380;
CZu = -0.00166;
CMu = 0.09677;
CXa = 0.30449;
CLa = 5.65160;
CMa = -7.48898;
CXwponto = 0;
CLwponto = 0; % Não tenho como saber ainda
CMwponto = 0;
CXq = -1.10793;
CLq = 54.02705;
CMq = -344.98052;
CYb = -0.11416;
Clb = 0.10998;
CNb = 0.17299;
CYp = 0.20362;
Clp = -0.74092;
CNp = -0.42465;
CYr = 0.35551;
Clr = -0.20002;
CNr = -0.53258;
CXd_a = -0.00519;
CYd_a = 0;
CZd_a = 0.00046;
Cld_a = 0.33103;
CMd_a = 0.00124;
CNd_a = 0.02907;
CXd_t = -0.03699;
CYd_t = 0; % meti a 0 porque não faz sentido
CZd_t = -1.21190;
Cld_t = 0;
CMd_t = -14.27522;
CNd_t = 0;
CXd_r = -0.00023;
CYd_r = 0.09253;
CZd_r = 0.00011;
Cld_r = -0.07738;
CMd_r = 0.00146;
CNd_r = -0.14062;

%%% DERIVADAS DIMENSIONAIS DE ESTABILIDADE %%
V = sqrt(u^2 + v^2 + w^2);
beta = asin(u/V);
alpha = atan(w/V);

Xu = rho*u0*S*sin(teta0)*Cw0 + 0.5*rho*S*u0*CXu;
Zu = -rho*u0*S*cos(teta0)*Cw0 + 0.5*rho*S*u0*CZu;
Mu = 0.5*rho*S*u0*c*CMu;
du = [Xu ; Zu ; Mu];

Xw = 0.5*rho*S*u0*CXa;
CZa = -(CLa + Cd1);
Zw = 0.5*rho*S*u0*CZa;
Mw = 0.5*rho*S*u0*CMa;
dw = [Xw ; Zw ; Mw];

Xwponto = 0.25*rho*S*c*CXwponto;
CZwponto = -CLwponto;
Zwponto = 0.25*rho*S*c*CZwponto;
Mwponto = 0.25*rho*S*c*c*CMwponto;
dwponto = [Xwponto ; Zwponto ; Mwponto];

Xq = 0.25*rho*S*u0*c*CXq;
CZq = -CLq;
Zq = 0.25*rho*S*u0*c*CZq;
Mq = 0.25*rho*S*u0*c*c*CMq;
dq = [Xq ; Zq ; Mq];

Yv = 0.25*rho*S*u0*CYb;
Lv = 0.5*rho*S*u0*beta*Clb;
Nv = 0.5*rho*S*u0*beta*CNb;
dv = [Yv ; Lv ; Nv];

Yp = 0.25*rho*u0*S*beta*CYp;
Lp = 0.25*rho*u0*S*beta*beta*Clp;
Np = 0.25*rho*u0*S*beta*beta*CNp;
dp = [Yp ; Lp ; Np];

Yr = 0.25*rho*S*u0*beta*CYr;
Lr = 0.25*rho*S*u0*beta*beta*Clr;
Nr = 0.25*rho*S*u0*beta*beta*CNr;
dr = [Yr ; Lr ; Nr];

%%% DERIVADAS DIMENSIONAIS CONTRLOLO EM RADIANOS%

Act_Ailerons = deg2rad(Act_Ailerons);
Act_Rear = deg2rad(Act_Rear);
Rudder = deg2rad(Rudder);

Xd_a = 0.5*rho*u0*u0*S*CXd_a;
Yd_a = 0.5*rho*u0*u0*S*CYd_a;
Zd_a = 0.5*rho*u0*u0*S*CZd_a;
Ld_a = 0.5*rho*u0*u0*S*beta*Cld_a;
Md_a = 0.5*rho*u0*u0*S*c*CMd_a;
Nd_a = 0.5*rho*u0*u0*S*beta*CNd_a;

Xd_t = 0.5*rho*u0*u0*S*CXd_t;
Yd_t = 0.5*rho*u0*u0*S*CYd_t;
Zd_t = 0.5*rho*u0*u0*S*CZd_t;
Ld_t = 0.5*rho*u0*u0*S*beta*Cld_t;
Md_t = 0.5*rho*u0*u0*S*c*CMd_t;
Nd_t = 0.5*rho*u0*u0*S*beta*CNd_t;

Xd_r = 0.5*rho*u0*u0*S*CXd_r;
Yd_r = 0.5*rho*u0*u0*S*CYd_r;
Zd_r = 0.5*rho*u0*u0*S*CZd_r;
Ld_r = 0.5*rho*u0*u0*S*beta*Cld_r;
Md_r = 0.5*rho*u0*u0*S*c*CMd_r;
Nd_r = 0.5*rho*u0*u0*S*beta*CNd_r;

%%% ALGURES CONDIÇÃO DE VENTILAÇÃO %%%

%%% THRUST %%%

Rpms_propeller = Rpms_motor/3;
diam = 0.335;

% Coefficients for KT 
KT_p1 = -39.575;
KT_p2 = 296.698;
KT_p3 = -977.420;
KT_p4 = 1855.793;
KT_p5 = -2236.302;
KT_p6 = 1772.128;
KT_p7 = -922.459;
KT_p8 = 303.597;
KT_p9 = -57.450;
KT_p10 = 5.070;

% KT function 
KT = @(x) KT_p1*x.^9 + KT_p2*x.^8 + KT_p3*x.^7 + KT_p4*x.^6 + KT_p5*x.^5 + KT_p6*x.^4 + KT_p7*x.^3 + KT_p8*x.^2 + KT_p9*x + KT_p10;
 
% Coefficients for kq 
Kq_p1 = 2.787;
Kq_p2 = -23.468;
Kq_p3 = 85.726;
Kq_p4 = -178.762;
Kq_p5 = 234.931;
Kq_p6 = -202.035;
Kq_p7 = 113.776;
Kq_p8 = -40.505;
Kq_p9 = 8.257;
Kq_p10 = -0.689;

% Kq function 
Kq = @(x) Kq_p1*x.^9 + Kq_p2*x.^8 + Kq_p3*x.^7 + Kq_p4*x.^6 + Kq_p5*x.^5 + Kq_p6*x.^4 + Kq_p7*x.^3 + Kq_p8*x.^2 + Kq_p9*x + Kq_p10;

Eff_p1 = -8627.603;
Eff_p2 = 61047.137;
Eff_p3 = -190004.846;
Eff_p4 = 341275.624;
Eff_p5 = -389694.485;
Eff_p6 = 293275.527;
Eff_p7 = -145425.792;
Eff_p8 = 45806.430;
Eff_p9 = -8313.959;
Eff_p10 = 662.694;
% Eff function 
Eff = @(x) Eff_p1*x.^9 + Eff_p2*x.^8 + Eff_p3*x.^7 + Eff_p4*x.^6 + Eff_p5*x.^5 + Eff_p6*x.^4 + Eff_p7*x.^3 + Eff_p8*x.^2 + Eff_p9*x + Eff_p10;

js = 60*V/(Rpms_propeller*diam);

kq = Kq(js);
kt = KT(js);
eff = Eff(js);

T = rho*(Rpms_propeller/60)^2*diam^4*kt;
P_propeller = T*V/eff;

%Calculate Power usage
P_mec=P_propeller/0.8;
Torque=P_mec/(Rpms_motor*2*pi/60);

%%% SOMATÓRIO %%%

X = du(1)*deltau + dw(1)*deltaw + dwponto(1)*wponto + dq(1)*q + Xd_a*Act_Ailerons + Xd_t*deltaAct_Rear + Xd_r*Rudder;
Y = dv(1)*v + dp(1)*p + dr(1)*r + Yd_a*Act_Ailerons + Yd_t*Act_Rear + Yd_r*Rudder;
Z = du(2)*deltau + dw(2)*deltaw + dwponto(2)*wponto + dq(2)*q + Zd_a*Act_Ailerons + Zd_t*deltaAct_Rear + Zd_r*Rudder;
L = dv(2)*v + dp(2)*p + dr(2)*r + Ld_a*Act_Ailerons + Ld_t*Act_Rear + Ld_r*Rudder;
M = du(3)*deltau + dw(3)*deltaw + dwponto(3)*wponto + dq(3)*q + Md_a*Act_Ailerons + Md_t*deltaAct_Rear + Md_r*Rudder;
N = dv(3)*v + dp(3)*p + dr(3)*r + Nd_a*Act_Ailerons + Nd_t*Act_Rear + Nd_r*Rudder;

%d = [dist_CGSG_CGASAS_X ; dist_CGSG_CGASAS_Y ; dist_CGSG_CGASAS_Z];
F = [X + T ; Y ; Z];
%M_1 = cross(d,F); não é necessário porque forças já estão no CG

M_2 = [L ; M ; N];

M_T = [0 ; 2.6 * T; 0]; % Momento só do Thrust, confia
Tau = M_2 + M_T; % Momentos livres + Momentos das Forças no CG do SG


%%% DERIVADAS %%%

R = [cosd(Pitch)*cosd(Yaw) cosd(Pitch)*sind(Yaw) -sind(Pitch);
    sind(Roll)*sind(Pitch)*cosd(Yaw)-cosd(Roll)*sind(Yaw) sind(Roll)*sind(Pitch)*sind(Yaw)+cosd(Roll)*cosd(Yaw) sind(Roll)*cosd(Pitch);
    cosd(Roll)*sind(Pitch)*cosd(Yaw)+sind(Roll)*sind(Yaw) cosd(Roll)*sind(Pitch)*sind(Yaw)-sind(Roll)*cosd(Yaw) cosd(Roll)*cosd(Pitch)]; % MATRIZ DE TRANSFERÊNCIA DE REFERENCIAL

%Balance of forces
p = p*pi/180;
q = q*pi/180;
r = r*pi/180;
ww = [p;q;r];

%W_inertial = m*g*[0;0;1];
W_inertial = 0;

uvw_dot = (F + R*W_inertial)/m - cross(ww, [u;v;w]);
VI_dot = R'*[u;v;w];

%Inertial angular velocities
Q = [1 sind(Roll)*tand(Pitch) cosd(Roll)*tand(Pitch);
     0 cosd(Roll) -sind(Roll);
     0 sind(Roll)/cosd(Pitch) cosd(Roll)/cosd(Pitch)];
eul = Q*ww;
Roll_dot = eul(1);
Pitch_dot = eul(2);
Yaw_dot = eul(3);

%Balance of moments

J = [Ixx 0 Ixz;
     0   Iyy 0;
     Ixz 0 Izz];
invJ = [-Izz/(Ixz^2 - Ixx*Izz),     0,  Ixz/(Ixz^2 - Ixx*Izz);
            0,                  1/Iyy,                      0;
       Ixz/(Ixz^2 - Ixx*Izz),     0,  -Ixx/(Ixz^2 - Ixx*Izz)];

wdot = invJ*(Tau - cross(ww,J*ww));

p_dot = wdot(1)*180/pi;
q_dot = wdot(2)*180/pi;
r_dot = wdot(3)*180/pi;

u_dot = uvw_dot(1);
v_dot = uvw_dot(2);
w_dot = uvw_dot(3);

Z_dot = VI_dot(3);

end