%Contributors: Maxwell Meador, Robert Reynoso
%Course number: ASEN 3801
%File name: QuadrotorEOM
%Created: 3/3/2026

function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)

    xe = var(1); 
    ye = var(2);
    ze = var(3);
    phi = var(4);
    theta = var(5);
    psi = var(6);
    ue = var(7);
    ve = var(8);
    we = var(9);
    p = var(10);
    q = var(11);
    r = var(12);
    
    Ix = I(1, 1);
    Iy = I(2, 2);
    Iz = I(3, 3);
    
    % Forces and moments
    L = -1*mu*sqrt(p^2+q^2+r^2)*p; 
    M = -1*mu*sqrt(p^2+q^2+r^2)*q; 
    N = -1*mu*sqrt(p^2+q^2+r^2)*r;
    
    Lc = 0; 
    Mc = 0; 
    Nc = 0;
    
    Xc = 0; 
    Yc = 0; 
    Zc = -9.81;
    
    % Trig
    c_theta = cos(theta); s_theta = sin(theta);
    c_psi = cos(psi);     s_psi = sin(psi);
    c_phi = cos(phi);     s_phi = sin(phi);

    % Kinematics
    xe_dot = (c_theta*c_psi)*ue + (s_phi*s_theta*c_psi - c_phi*s_psi)*ve + (c_phi*s_theta*c_psi + s_phi*s_psi)*we;
    ye_dot = (c_theta*s_psi)*ue + (s_phi*s_theta*s_psi + c_phi*c_psi)*ve + (c_phi*s_theta*s_psi - s_phi*c_psi)*we;
    ze_dot = (-s_theta)*ue + (s_phi*c_theta)*ve + (c_phi*c_theta)*we;
    
    phi_dot   = p + (sin(phi)*tan(theta))*q + (cos(phi)*tan(theta))*r;
    theta_dot = (cos(phi))*q - (sin(phi))*r;
    psi_dot   = (sin(phi)*sec(theta))*q + (cos(phi)*sec(theta))*r;
    
    % Drag
    Va = sqrt(ue^2 + ve^2 + we^2); 
    X = -1 * nu * Va * ue;
    Y = -1 * nu * Va * ve;
    Z = -1 * nu * Va * we;

    % Dynamics
    u_dot = r*ve - q*we - g*sin(theta) + X/m + Xc/m;
    v_dot = p*we - r*ue + g*cos(theta)*sin(phi) + Y/m + Yc/m;
    w_dot = q*ue - p*ve + g*cos(theta)*cos(phi) + Z/m + Zc/m;
    
    p_dot = ((Iy - Iz)/Ix)*q*r + L/Ix + Lc/Ix;
    q_dot = ((Iz - Ix)/Iy)*p*r + M/Iy + Mc/Iy;
    r_dot = ((Ix - Iy)/Iz)*p*q + N/Iz + Nc/Iz;
    
    % State Vector Deriv Return
    var_dot = [xe_dot; ye_dot; ze_dot; phi_dot; theta_dot; psi_dot; u_dot; v_dot; w_dot; p_dot; q_dot; r_dot];
end
