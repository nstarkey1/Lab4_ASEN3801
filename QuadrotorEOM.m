function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
% Quadrotor nonlinear equations of motion
% var = [xE; yE; zE; phi; theta; psi; u; v; w; p; q; r]
% motor_forces = [f1; f2; f3; f4]


    %% States
    xE    = var(1);
    yE    = var(2);
    zE    = var(3);
    phi   = var(4);
    theta = var(5);
    psi   = var(6);
    u     = var(7);
    v     = var(8);
    w     = var(9);
    p     = var(10);
    q     = var(11);
    r     = var(12);

    %% Inertia values
    Ix = I(1,1);
    Iy = I(2,2);
    Iz = I(3,3);

    %% Motor forces
    f1 = motor_forces(1);
    f2 = motor_forces(2);
    f3 = motor_forces(3);
    f4 = motor_forces(4);

    %% Control force and moments from rotors
    % From lab handout:
    % Zc = -f1 - f2 - f3 - f4
    % Lc = d/sqrt(2) * (-f1 - f2 + f3 + f4)
    % Mc = d/sqrt(2) * ( f1 - f2 - f3 + f4)
    % Nc = km * (f1 - f2 + f3 - f4)
    Zc = -(f1 + f2 + f3 + f4);
    Lc = (d/sqrt(2)) * (-f1 - f2 + f3 + f4);
    Mc = (d/sqrt(2)) * ( f1 - f2 - f3 + f4);
    Nc = km * (f1 - f2 + f3 - f4);

    %% Trig shorthand
    cphi = cos(phi);
    sphi = sin(phi);
    cth  = cos(theta);
    sth  = sin(theta);
    cpsi = cos(psi);
    spsi = sin(psi);

    %% Position kinematics (body velocities -> inertial position rates)
    xE_dot = cth*cpsi*u + (sphi*sth*cpsi - cphi*spsi)*v + (cphi*sth*cpsi + sphi*spsi)*w;
    yE_dot = cth*spsi*u + (sphi*sth*spsi + cphi*cpsi)*v + (cphi*sth*spsi - sphi*cpsi)*w;
    zE_dot = -sth*u    + sphi*cth*v                         + cphi*cth*w;

    %% Euler angle kinematics
    phi_dot   = p + sphi*tan(theta)*q + cphi*tan(theta)*r;
    theta_dot =     cphi*q            - sphi*r;
    psi_dot   =     (sphi/cth)*q      + (cphi/cth)*r;

    %% Aerodynamic drag forces
    % nu = aerodynamic force coefficient
    Va = sqrt(u^2 + v^2 + w^2);
    X = -nu * Va * u;
    Y = -nu * Va * v;
    Z = -nu * Va * w;

    %% Aerodynamic drag moments
    % mu = aerodynamic moment coefficient
    L = -mu * abs(p) * p;
    M = -mu * abs(q) * q;
    N = -mu * abs(r) * r;

    %% Translational dynamics in body frame
    u_dot = r*v - q*w - g*sin(theta)          + X/m;
    v_dot = p*w - r*u + g*cth*sin(phi)        + Y/m;
    w_dot = q*u - p*v + g*cth*cphi            + Z/m + Zc/m;

    %% Rotational dynamics in body frame
    p_dot = ((Iy - Iz)/Ix)*q*r + (L + Lc)/Ix;
    q_dot = ((Iz - Ix)/Iy)*p*r + (M + Mc)/Iy;
    r_dot = ((Ix - Iy)/Iz)*p*q + (N + Nc)/Iz;

    %% Return state derivative
    var_dot = [xE_dot;
               yE_dot;
               zE_dot;
               phi_dot;
               theta_dot;
               psi_dot;
               u_dot;
               v_dot;
               w_dot;
               p_dot;
               q_dot;
               r_dot];
end