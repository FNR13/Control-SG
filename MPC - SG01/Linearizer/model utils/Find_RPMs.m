Act_Ailerons = 0;
Act_Rear = 2.35;
Rudder = 0;
u0 = 10.2823; w0 = -0.2351; p0 = 0; q0 = 0; r0 = 0;
Pitch0 = -1.31; Roll0 = 0; Yaw0 = 0;
z_cm = 50;
struts = 1;

% Chama a função para encontrar as RPMs de equilíbrio
Rpms_equilibrio = Find_Rpms_Equilibrium(Act_Ailerons, Act_Rear, Rudder, u0, w0, p0, q0, r0, Pitch0, Roll0, Yaw0, z_cm, struts);
fprintf("RPMs de Equilíbrio: %f\n", Rpms_equilibrio)

function Rpms_equilibrium = Find_Rpms_Equilibrium(Act_Ailerons, Act_Rear, Rudder, u0, w0, p0, q0, r0, Pitch0, Roll0, Yaw0, z_cm, struts)
    % Constantes
    g = 9.81;  % Aceleração gravitacional (m/s^2)
    rho = 1026.00; % Densidade da água (kg/m^3)
    diam = 0.335; % Diâmetro (m)

    % Forças e momentos alvo (equilíbrio)
    Target_Forces = [0; 0; 0]; 
    Target_Moments = [0; 0; 0]; 

    % Limites das RPMs e tolerância
    Rpms_min = 0;
    Rpms_max = 5000; 
    tolerance = 1e-6;  % Tolerância ajustada
    
    % Opções para otimização
    options = optimset('Display', 'iter', 'TolFun', tolerance, 'TolX', tolerance, 'Algorithm', 'quasi-newton');
    
    % Otimização utilizando fminunc
    Rpms_equilibrium = fminunc(@(Rpms_motor) Equilibrium_Error(Rpms_motor, Act_Ailerons, Act_Rear, Rudder, u0, w0, p0, q0, r0, Pitch0, Roll0, Yaw0, z_cm, struts, Target_Forces, Target_Moments), (Rpms_min + Rpms_max)/2, options);
end

function error = Equilibrium_Error(Rpms_motor, Act_Ailerons, Act_Rear, Rudder, u, w, p, q, r, Pitch, Roll, Yaw, z_cm, struts, Target_Forces, Target_Moments)
    % Calcula as forças e momentos usando a função SG01_Flight_Dynamics
    [u_dot, v_dot, w_dot, p_dot, q_dot, r_dot, Pitch_dot, Roll_dot, Yaw_dot, Z_dot, Torque, ~] = SG01_Flight_Dynamics(Act_Ailerons, Act_Rear, Rudder, Rpms_motor, u, 0, w, 0, p, q, r, Pitch, Roll, Yaw, z_cm, struts);
    
    % Monta os vetores de forças e momentos
    Forces = [u_dot; v_dot; w_dot];
    Moments = [p_dot; q_dot; r_dot];
    
    % Exibir as forças e momentos calculados a cada iteração (diagnóstico)
    fprintf('Forças calculadas: [%f, %f, %f]\n', Forces(1), Forces(2), Forces(3));
    fprintf('Momentos calculados: [%f, %f, %f]\n', Moments(1), Moments(2), Moments(3));
    
    % Ajuste para maior penalização de momentos
    moment_penalty_factor = 1000;  % Aumentando a penalização dos momentos
    
    % Calcula os erros em relação às forças e momentos alvo com penalização de momentos
    error_forces = norm(Forces - Target_Forces);  % Erro das forças
    error_moments = moment_penalty_factor * norm(Moments - Target_Moments);  % Erro dos momentos com maior penalização
    
    % Soma dos erros para otimização
    error = error_forces + error_moments;
end
