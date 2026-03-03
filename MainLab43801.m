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
I = diag([Iz Iy Iz]);
nu = 1*10^-3;
mu = 2*10^-6;
g = 9.81; 
t = rt.rt_estim.time;
motor_forces = [0;0;0;0];


var = rt.rt_estim.signals.values;

var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces);