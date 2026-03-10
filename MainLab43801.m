clear;
clc;
close all;

rt = importdata("RSdata_nocontrol.mat");
m = 0.068;
d = 0.060;
km = 0.0024;
Ix = 5.8*10^-5;
Iy = 7.2*10^-5;
Iz = 1*10^-4;
I = diag([Ix Iy Iz]);
nu = 1*10^-3;
mu = 2*10^-6;
g = 9.81; 
t_span = rt.rt_estim.time;
motor_forces = [0;0;0;0];

initial_conditions = rt.rt_estim.signals.values(1, :)'; 

ode_func = @(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces);

%% 1.3

while t_span < 10
[t_out, statevec] = ode45(ode_func, t_span, initial_conditions);
end


