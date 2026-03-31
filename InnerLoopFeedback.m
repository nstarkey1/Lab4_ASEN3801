
%question 3.2
function [Fc, Gc] = InnerLoopFeedback(var)
m = 0.068;
g = 9.81;

%roll gains
Kp = 5.8e-4;
Kphi = 9.28e-4;

%pitch gains
Kq = 7.2e-4;
Ktheta = 1.15e-3;

K = 0.004; %Nm/rad, angular feedback control from problem 2.3

%states
phi = var(4);
theta = var(5);
p = var (10);
q = var(11);
r = var(12);

%control force
Fc = [0;0; -m*g];

%control moment
Lc = -Kphi*phi - Kp*p;
Mc = -Ktheta*theta - Kq*q;
Nc = -K*r;
Gc = [Lc; Mc; Nc];
end 