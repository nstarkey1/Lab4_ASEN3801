function motor_forces = ComputeMotorForces(Fc, Gc, d, km)

    %% Control F and M extraction
    Zc = Fc(3);    % (N)
    Lc = Gc(1);    % roll (N*m)
    Mc = Gc(2);    % pitch (N*m)
    Nc = Gc(3);    % yaw (N*m)

    %% Control Matrix

    s = sqrt(2);

    M = [ -1,       -1,       -1,       -1;
          -d/s,     -d/s,      d/s,      d/s;
           d/s,     -d/s,     -d/s,      d/s;
           km,      -km,       km,      -km ];

    %% Solve for forces
    control_vec   = [Zc; Lc; Mc; Nc];
    motor_forces  = M \ control_vec;  
end
