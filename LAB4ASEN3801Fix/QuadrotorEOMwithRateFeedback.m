function var_dot = QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu)
    %% Params
    d  = 0.060; 
    km = 0.0024; %(N*m/N)

    %% Control force and mom from state
    [Fc, Gc] = RotationDerivativeFeedback(var, m, g);

    %% Desired [Fc,Gc] to motor forces
    motor_forces = ComputeMotorForces(Fc, Gc, d, km);

    %% Nonlinear EOM
    var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces);
end
