Act_Ailerons = 0;
Act_Rear = 2.35;
Rudder = 0;
u0 = 10.285; w0 = 0; p0 = 0; q0 = 0; r0 = 0;
Pitch0 = -1.31; Roll0 = 0; Yaw0 = 0;
z_cm = 50;
struts = 1;

Rpms_equilibrio = Find_Rpms_Equilibrium(Act_Ailerons, Act_Rear, Rudder, u0, w0, p0, q0, r0, Pitch0, Roll0, Yaw0, z_cm, struts);
fprintf("%f",Rpms_equilibrio)

function Rpms_equilibrium = Find_Rpms_Equilibrium(Act_Ailerons, Act_Rear, Rudder, u0, w0, p0, q0, r0, Pitch0, Roll0, Yaw0, z_cm, struts)

    g = 9.81;
    rho = 1026.00;
    diam = 0.335;

    Target_Forces = [0; 0; 0]; 
    Target_Moments = [0; 0; 0]; 

    Rpms_min = 0;
    Rpms_max = 5000; 
    tolerance = 1e-3; 

    options = optimset('Display', 'iter', 'TolFun', tolerance);
    Rpms_equilibrium = fminbnd(@(Rpms_motor) Equilibrium_Error(Rpms_motor, Act_Ailerons, Act_Rear, Rudder, u0, w0, p0, q0, r0, Pitch0, Roll0, Yaw0, z_cm, struts, Target_Forces, Target_Moments), Rpms_min, Rpms_max, options);
end

function error = Equilibrium_Error(Rpms_motor, Act_Ailerons, Act_Rear, Rudder, u, w, p, q, r, Pitch, Roll, Yaw, z_cm, struts, Target_Forces, Target_Moments)

    [u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, Pitch_dot, Roll_dot, Yaw_dot, Z_dot, Torque, ~] = SG01_Flight_Dynamics(Act_Ailerons, Act_Rear, Rpms_motor,Rudder, u, 0, w, p, q, r, Pitch, Roll, Yaw, z_cm);

    Forces = [u_dot; v_dot; w_dot];
    Moments = [p_dot; q_dot; r_dot];

    error_forces = norm(Forces - Target_Forces);
    error_moments = norm(Moments - Target_Moments);

    error = error_forces + error_moments;
end

