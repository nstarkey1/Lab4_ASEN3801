function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)
    %% Unpacking
    p = var(10);   % roll  rate (rad/s)
    q = var(11);   % pitch rate (rad/s)
    r = var(12);   % yaw   rate (rad/s)

    %% Rate Feedback Gain
    K_rate = 0.004;

    %% Control Force
    Xc = 0;
    Yc = 0;
    Zc = -m * g;

    Fc = [Xc; Yc; Zc];

    %% Control Moments
    % Proportional to angular rate, opposite sign
    Lc = -K_rate * p;
    Mc = -K_rate * q;
    Nc = -K_rate * r;

    Gc = [Lc; Mc; Nc];
end
