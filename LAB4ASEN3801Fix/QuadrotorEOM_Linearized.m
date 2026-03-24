function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)

    %% Unpacking

    Ix = I(1,1);
    Iy = I(2,2);
    Iz = I(3,3);


    % delta_xE    = var(1); 
    % delta_yE    = var(2);
    % delta_zE    = var(3);
    delta_phi   = var(4);
    delta_theta = var(5);
    % delta_psi   = var(6);
    delta_u     = var(7);
    delta_v     = var(8);
    delta_w     = var(9);
    delta_p     = var(10);
    delta_q     = var(11);
    delta_r     = var(12);

    % deltaFc = [deltaXc; deltaYc; deltaZc]
    % deltaGc = [deltaLc; deltaMc; deltaNc]

    deltaXc = deltaFc(1);
    deltaYc = deltaFc(2);
    deltaZc = deltaFc(3);
    deltaLc = deltaGc(1);
    deltaMc = deltaGc(2);
    deltaNc = deltaGc(3);

    %% Kinematics
    delta_xE_dot = delta_u;
    delta_yE_dot = delta_v;
    delta_zE_dot = delta_w;

    delta_phi_dot   = delta_p;
    delta_theta_dot = delta_q;
    delta_psi_dot   = delta_r;

    delta_u_dot = -g * delta_theta + deltaXc/m;
    delta_v_dot =  g * delta_phi   + deltaYc/m;
    delta_w_dot =                    deltaZc/m;

    %% Dynamics

    delta_p_dot = deltaLc / Ix;
    delta_q_dot = deltaMc / Iy;
    delta_r_dot = deltaNc / Iz;

    %% Assemble
    var_dot = [delta_xE_dot;
               delta_yE_dot;
               delta_zE_dot;
               delta_phi_dot;
               delta_theta_dot;
               delta_psi_dot;
               delta_u_dot;
               delta_v_dot;
               delta_w_dot;
               delta_p_dot;
               delta_q_dot;
               delta_r_dot];
end
