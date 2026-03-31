function var_dot = QuadrotorEOM_Nonlinear_InnerLoop(t, var, g, m, I, d, km, nu, mu)

    % States
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

    % Inertia values
    Ix = I(1,1);
    Iy = I(2,2);
    Iz = I(3,3);

    % Controller
    [Fc, Gc] = InnerLoopFeedback(var);

    % Convert force/moments to motor forces
    motor_forces = ComputeMotorForces(Fc, Gc, d, km);

    f1 = motor_forces(1);
    f2 = motor_forces(2);
    f3 = motor_forces(3);
    f4 = motor_forces(4);

    % Control force/moments from motor forces
    Zc = -(f1 + f2 + f3 + f4);
    Lc = (d/sqrt(2)) * (-f1 - f2 + f3 + f4);
    Mc = (d/sqrt(2)) * ( f1 - f2 - f3 + f4);
    Nc = km          * ( f1 - f2 + f3 - f4);

    % Trig
    cphi = cos(phi);
    sphi = sin(phi);
    cth  = cos(theta);
    sth  = sin(theta);
    cpsi = cos(psi);
    spsi = sin(psi);
    tth  = tan(theta);

    % Aerodynamic drag
    Va = sqrt(u^2 + v^2 + w^2);

    Xaero = -nu * Va * u;
    Yaero = -nu * Va * v;
    Zaero = -nu * Va * w;

    Laero = -mu * abs(p) * p;
    Maero = -mu * abs(q) * q;
    Naero = -mu * abs(r) * r;

    % Kinematics
    xE_dot =  (cth*cpsi)*u + (sphi*sth*cpsi - cphi*spsi)*v + (cphi*sth*cpsi + sphi*spsi)*w;
    yE_dot =  (cth*spsi)*u + (sphi*sth*spsi + cphi*cpsi)*v + (cphi*sth*spsi - sphi*cpsi)*w;
    zE_dot = -(sth)*u      + (sphi*cth)*v                  + (cphi*cth)*w;

    phi_dot   = p + sphi*tth*q + cphi*tth*r;
    theta_dot = cphi*q - sphi*r;
    psi_dot   = (sphi/cth)*q + (cphi/cth)*r;

    % Dynamics
    u_dot = r*v - q*w - g*sth      + Xaero/m;
    v_dot = p*w - r*u + g*cth*sphi + Yaero/m;
    w_dot = q*u - p*v + g*cth*cphi + Zaero/m + Zc/m;

    p_dot = ((Iy - Iz)/Ix)*q*r + (Laero + Lc)/Ix;
    q_dot = ((Iz - Ix)/Iy)*p*r + (Maero + Mc)/Iy;
    r_dot = ((Ix - Iy)/Iz)*p*q + (Naero + Nc)/Iz;

    % Assemble
    var_dot = [xE_dot;yE_dot;zE_dot;phi_dot;theta_dot;psi_dot;u_dot;v_dot;w_dot;p_dot;q_dot;r_dot];
end