clear;
clc;
close all;

rt = importdata("RSdata_nocontrol.mat");

% Parameters
m  = 0.068;
d  = 0.060;
km = 0.0024;
Ix = 5.8e-5;
Iy = 7.2e-5;
Iz = 1.0e-4;
I  = diag([Ix Iy Iz]);
nu = 1e-3;
mu = 2e-6;
g  = 9.81;

t_span = [0 10];

% Hover trim motor thrusts
motor_forces = (m*g/4)*ones(4,1);

% Initial conditions
initial_conditions = rt.rt_estim.signals.values(1, :)';
initial_conditions = [0;0;0;0;0;0;0;0;0;0;0;0];

% ODE function
ode_func = @(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces);

% Integrate
[t_out, statevec] = ode45(ode_func, t_span, initial_conditions);

% Extract states
x     = statevec(:,1);
y     = statevec(:,2);
z     = statevec(:,3);
phi   = statevec(:,4);
theta = statevec(:,5);
psi   = statevec(:,6);
u     = statevec(:,7);
v     = statevec(:,8);
w     = statevec(:,9);
p     = statevec(:,10);
q     = statevec(:,11);
r     = statevec(:,12);

% Convert for plotting
aircraft_state_array = statevec';

% Control inputs (none for now)
control_input_array = zeros(4,length(t_out));

% Plot
fig = [1 2 3 4 5];
col = 'b';

PlotAircraftSim(t_out, aircraft_state_array, control_input_array, fig, col)