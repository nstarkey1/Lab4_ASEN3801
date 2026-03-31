function var_dot = QuadrotorEOM_Linearized_InnerLoop(t, var, g, m, I)

    % Inertia values
    Ix = I(1,1);
    Iy = I(2,2);
    Iz = I(3,3);

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

    % Controller
    [Fc, Gc] = InnerLoopFeedback(var);

    Lc = Gc(1);
    Mc = Gc(2);
    Nc = Gc(3);

    % Linearized closed-loop equations about hover
    xE_dot    = u;
    yE_dot    = v;
    zE_dot    = w;

    phi_dot   = p;
    theta_dot = q;
    psi_dot   = r;

    u_dot     = -g*theta;
    v_dot     =  g*phi;
    w_dot     =  0;

    p_dot     = Lc/Ix;
    q_dot     = Mc/Iy;
    r_dot     = Nc/Iz;

    var_dot = [xE_dot; yE_dot; zE_dot; phi_dot; theta_dot; psi_dot; u_dot; v_dot; w_dot; p_dot;q_dot;r_dot];
end