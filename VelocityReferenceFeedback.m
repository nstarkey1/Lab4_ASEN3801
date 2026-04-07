
%Contributors: Anais De Jesus
%Course numer: ASEN 3801
%File Name: VelocityReferenceFeedback
%created: 4/3/26


function [Fc, Gc] = VelocityReferenceFeedback(t,var)
%inputs: current time, 12x1 state vector
%outputs: control vectors Fc and Gc
%methodology: use definitions to calculate control vectors Fc and Gc

%v = 1/2 = 0.5 m/s
%Parameters
m = 0.068;
g = 9.81;

%Innerloop gains from 3.1
Kp = 5.8e-4; %roll
Kphi = 9.28e-4; %roll
Kq = 7.2e-4; %pitch
Ktheta = 1.152e-3; %pitch
K = 0.004; %%Nm/rad, angular feedback control from problem 2.3

%Outerloop gains from 3.5
K3_lat = 4.086e-5;
K3_long = 5.073e-5;

%choose which direction to test (easier to do them seperately)
mode = 'longitudinal'; 
%mode = 'lateral'

%states
phi = var(4);
theta = var(5);
u = var(7);
v = var(8);
p = var(10);
q = var(11);
r = var(12);

u_ref = 0;
v_ref = 0;

%this is a step input in velocity thats ruens off at 2s
if t <= 2
    if strcmp(mode, 'longitudinal')
        u_ref = 0.5;
    elseif strcmp(mode, 'lateral')
        v_ref = 0.5;
    end
end

%control force
Fc = [0; 0; -m*g];

%control moments
Lc = -Kphi*phi - Kp*p + K3_lat*(v_ref - v); %lateral outer loop
Mc = -Ktheta*theta - Kq*q - K3_long*(u_ref - u); %longitudinal outer loop
Nc = -K*r;

Gc = [Lc; Mc; Nc];
end