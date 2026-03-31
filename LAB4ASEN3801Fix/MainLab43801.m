%% MainLab43801.m
% Done for tasks 1 and 2 but im pretty sure theres stuff broken still
clear; clc; close all;

%% Parameters
m  = 0.068;          % (kg)
d  = 0.060;          % (m)
km = 0.0024;         % (N*m / N)
Ix = 5.8e-5;         % (kg*m^2)
Iy = 7.2e-5;         % (kg*m^2)
Iz = 1.0e-4;         % (kg*m^2)
I  = diag([Ix Iy Iz]); % inertia matrix
nu = 1e-3;           % translational drag (N/(m/s)^2)
mu = 2e-6;           % rotational drag (N*m/(rad/s)^2)
g  = 9.81;           % (m/s^2)
motor_forces_hover = (m*g/4) * ones(4,1);
t_span = [0 10];  

%% Plots 
%Select either true or false for each task you want plots for
plots12 = true;
plots14 = true;
plots15 = true;
plots21 = true;
plots22 = true;
plots24 = true;
plots25 = true;

%% 1.2 (Hover trim)
IC_hover = zeros(12,1);   % all states 0 for hover
% nu=0, mu=0
ode_hover_nodrag = @(t, var) QuadrotorEOM(t, var, g, m, I, d, km, 0, 0, motor_forces_hover);
[t12, X12] = ode45(ode_hover_nodrag, t_span, IC_hover);

s2 = sqrt(2);
Zc_hover = -(sum(motor_forces_hover));
Lc_hover = (d/s2)*(-motor_forces_hover(1) - motor_forces_hover(2) + motor_forces_hover(3) + motor_forces_hover(4));
Mc_hover = (d/s2)*( motor_forces_hover(1) - motor_forces_hover(2) - motor_forces_hover(3) + motor_forces_hover(4));
Nc_hover = km*(motor_forces_hover(1) - motor_forces_hover(2) + motor_forces_hover(3) - motor_forces_hover(4));

ctrl12 = repmat([Zc_hover; Lc_hover; Mc_hover; Nc_hover], 1, length(t12));

if plots12
    PlotAircraftSim(t12, X12', ctrl12, [1 2 3 4 5 6], 'b');
    sgtitle_figs([1 2 3 4 5 6], 'Task 1.2: Hover trim (no drag)');
end
% All states should be at zero

%% 1.2 (Hover trim w drag)
ode_hover_drag = @(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces_hover);
[t13, X13] = ode45(ode_hover_drag, t_span, IC_hover);

ctrl13 = repmat([Zc_hover; Lc_hover; Mc_hover; Nc_hover], 1, length(t13));

if plots12
    PlotAircraftSim(t13, X13', ctrl13, [11 12 13 14 15 16], 'b');
    sgtitle_figs([11 12 13 14 15 16], 'Task 1.3: Hover trim (with drag)');
end
% Should be 0 still if working

%% 1.4a (Const Vel. Trim w 5 m/s east, psi = 0)
u_trim = 5.0;
Va_trim = u_trim;

% Trim pitch angle
theta_trim_a = asin(-nu * u_trim^2 / (m * g));
phi_trim_a = 0; 

% Trim thrust
Zc_trim_a = -m * g / cos(theta_trim_a);
f_trim_a  = -Zc_trim_a / 4; 
motor_forces_trim_a = f_trim_a * ones(4,1);

IC_trim_a = [0; 0; 0; phi_trim_a; theta_trim_a; 0; u_trim; 0; 0; 0; 0; 0];
ode_trim_a = @(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces_trim_a);
[t14a, X14a] = ode45(ode_trim_a, t_span, IC_trim_a);

Zc14a = -(4*f_trim_a); Lc14a = 0; Mc14a = 0; Nc14a = 0;
ctrl14a = repmat([Zc14a; Lc14a; Mc14a; Nc14a], 1, length(t14a));

if plots14
    PlotAircraftSim(t14a, X14a', ctrl14a, [21 22 23 24 25 26], 'b');
    sgtitle_figs([21 22 23 24 25 26], 'Task 1.4a: 5 m/s East trim, psi=0');
end
% The only thing that should change I think is xE which seems to be
% increasing linearly at 5 m/s

%% 1.4b (Const Vel. Trim w 5 m/s east, psi = 90)
% Basically same as 1.4 a but swapping u for v
v_trim = 5.0;
Va_trim_b  = v_trim;

% Trim roll angle
phi_trim_b = asin(-nu * v_trim^2 / (m * g));
theta_trim_b = 0;

% Trim thrust
Zc_trim_b = -m * g * cos(phi_trim_b);
f_trim_b = -Zc_trim_b / 4;
motor_forces_trim_b = f_trim_b * ones(4,1);

IC_trim_b = [0; 0; 0; phi_trim_b; theta_trim_b; pi/2; 0; v_trim; 0; 0; 0; 0];
ode_trim_b = @(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces_trim_b);
[t14b, X14b] = ode45(ode_trim_b, t_span, IC_trim_b);

Zc14b = -(4*f_trim_b);
ctrl14b = repmat([Zc14b; 0; 0; 0], 1, length(t14b));

if plots14
    PlotAircraftSim(t14b, X14b', ctrl14b, [31 32 33 34 35 36], 'b');
    sgtitle_figs([31 32 33 34 35 36], 'Task 1.4b: 5 m/s East trim, psi=90 deg');
end
% Same as 1.4a but now its yE that increases

%% 1.5 (Hover stability)
% roll perturbation from hover
IC_perturb = zeros(12,1);
IC_perturb(4) = deg2rad(5);

ode_perturb = @(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces_hover);
[t15, X15] = ode45(ode_perturb, t_span, IC_perturb);

ctrl15 = repmat([Zc_hover; 0; 0; 0], 1, length(t15));

if plots15
    PlotAircraftSim(t15, X15', ctrl15, [41 42 43 44 45 46], 'b');
    sgtitle_figs([41 42 43 44 45 46], 'Task 1.5: Hover stability (simulated, +5 deg roll)');
end

% Hardware data
rt = importdata('RSdata_nocontrol.mat');
t_hw = rt.rt_estim.time;                     % time vector (s)
X_hw = rt.rt_estim.signals.values';          % (12 x n) state array

% Plot hardware data overlaid on the same figures (red) for comparison
ctrl_hw = zeros(4, length(t_hw));   % no control inputs in the hardware run

if plots15
    PlotAircraftSim(t_hw, X_hw, ctrl_hw, [41 42 43 44 45 46], 'r');
    figure(42)
    legend('Simulation', 'Hardware', 'Location', 'best', 'AutoUpdate', 'off');
end
% Hover looks unstable since the angles grow I think prolly double check
% this though

%% 2.1 (Nonlinear eq sim)
cases_21 = {
    4,  deg2rad(5),   '+5 deg roll';        % a
    5,  deg2rad(5),   '+5 deg pitch';       % b
    6,  deg2rad(5),   '+5 deg yaw';         % c
    10, 0.1,          '+0.1 rad/s roll rate';   % d
    11, 0.1,          '+0.1 rad/s pitch rate';  % e
    12, 0.1,          '+0.1 rad/s yaw rate';    % f
};

for k = 1:length(cases_21)
    state_idx = cases_21{k,1};
    deviation = cases_21{k,2};
    label     = cases_21{k,3};

    % All zero but perturbed IC
    IC_k = zeros(12,1);
    IC_k(state_idx) = deviation;

    ode_k = @(t, var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces_hover);
    [t_k, X_k] = ode45(ode_k, t_span, IC_k);

    % Control
    ctrl_k = repmat([Zc_hover; 0; 0; 0], 1, length(t_k));

    % Figure numbers: block of 6 per case, offset by case number
    figs_k = (100 + k*10) + (1:6);
    
    if plots21    
        PlotAircraftSim(t_k, X_k', ctrl_k, figs_k, 'b');
        sgtitle_figs(figs_k, sprintf('Task 2.1 Case %s: %s (NL)', char('a'+k-1), label));
    end
    
    % Store for Task 2.2
    t21{k}  = t_k;
    X21{k}  = X_k;
    figs21{k} = figs_k;
end
% EXPECTED:
% a-c angles look like diverging
% d-f angular rate change causes changing angles and position? Double chek

%% 2.2 (Linearized EOM)
deltaFc_hover = [0; 0; 0];   
deltaGc_hover = [0; 0; 0]; 

for k = 1:length(cases_21)
    state_idx = cases_21{k,1};
    deviation = cases_21{k,2};
    label     = cases_21{k,3};

    % Same initial condition as Task 2.1
    IC_k = zeros(12,1);
    IC_k(state_idx) = deviation;

    % LINEARIZED EOM
    ode_lin_k = @(t, var) QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc_hover, deltaGc_hover);
    [t_lin_k, X_lin_k] = ode45(ode_lin_k, t_span, IC_k);

    % Control
    ctrl_lin_k = repmat([Zc_hover; 0; 0; 0], 1, length(t_lin_k));
    
    if plots22
        % Overlay
        PlotAircraftSim(t_lin_k, X_lin_k', ctrl_lin_k, figs21{k}, 'r');
        figure(figs21{k}(1));
        legend('Nonlinear', 'Linearized', 'Location', 'best', 'AutoUpdate', 'off');
    end
end

%% 2.5 (Rate Feedback)
cases_25 = {4, 5, 6};

for idx = 1:length(cases_25)
    k = cases_25{idx};
    state_idx = cases_21{k,1};
    deviation = cases_21{k,2};
    label = cases_21{k,3};

    % Initial condition
    IC_k = zeros(12,1);
    IC_k(state_idx) = deviation;

    % Rate Feedback EOM ode call
    ode_ctrl = @(t, var) QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu);
    [t_ctrl, X_ctrl] = ode45(ode_ctrl, t_span, IC_k);

    ctrl_ctrl = zeros(4, length(t_ctrl));
    for ii = 1:length(t_ctrl)
        [Fc_ii, Gc_ii]    = RotationDerivativeFeedback(X_ctrl(ii,:)', m, g);
        mf_ii             = ComputeMotorForces(Fc_ii, Gc_ii, d, km);
        ctrl_ctrl(1,ii)   = Fc_ii(3);          % Zc
        ctrl_ctrl(2:4,ii) = Gc_ii;             % Lc, Mc, Nc
    end

    % Uncontrolled from 2.1 for comp
    t_unc   = t21{k};
    X_unc   = X21{k};
    ctrl_unc = repmat([Zc_hover; 0; 0; 0], 1, length(t_unc));
    
    figs_25k = (300 + idx*10) + (1:6);
    
    if plots25
        % Blue: uncontrolled nonlinear
        PlotAircraftSim(t_unc, X_unc', ctrl_unc, figs_25k, 'b');
        % Green: controlled nonlinear
        PlotAircraftSim(t_ctrl, X_ctrl', ctrl_ctrl, figs_25k, 'g');
        
        figure(figs_25k(1));
        legend('Uncontrolled', 'Rate Feedback', 'Location', 'best', 'AutoUpdate', 'off');
        sgtitle_figs(figs_25k, sprintf('Task 2.5 Case %s: %s', char('d'+idx-1), label));
     
        % Motor force comp
        figure(350 + idx);
        
        % Uncontrolled: constant hover thrust on all motors
        subplot(4,1,1); h_unc = plot(t_unc, (m*g/4)*ones(size(t_unc)), 'b', 'LineWidth', 1.5); hold on;
        subplot(4,1,2); plot(t_unc, (m*g/4)*ones(size(t_unc)), 'b', 'LineWidth', 1.5); hold on;
        subplot(4,1,3); plot(t_unc, (m*g/4)*ones(size(t_unc)), 'b', 'LineWidth', 1.5); hold on;
        subplot(4,1,4); plot(t_unc, (m*g/4)*ones(size(t_unc)), 'b', 'LineWidth', 1.5); hold on;
    end
    
    % Controlled
    mf_ctrl_all = zeros(4, length(t_ctrl));
    for ii = 1:length(t_ctrl)
        [Fc_ii, Gc_ii]    = RotationDerivativeFeedback(X_ctrl(ii,:)', m, g);
        mf_ctrl_all(:,ii) = ComputeMotorForces(Fc_ii, Gc_ii, d, km);
    end
    
    if plots25
        labels_mf = {'f_1 (N)', 'f_2 (N)', 'f_3 (N)', 'f_4 (N)'};
        for mi = 1:4
            subplot(4,1,mi);

            h_ctrl = plot(t_ctrl, mf_ctrl_all(mi,:), 'g', 'LineWidth', 1.5); hold on;
            ylabel(labels_mf{mi}); grid on;
            
            if mi == 1
                title(sprintf('Task 2.5 Case %s: Motor Forces', char('d'+idx-1)));

                legend([h_unc, h_ctrl], 'Uncontrolled', 'Rate Feedback', 'Location', 'best');
            end
            if mi == 4; xlabel('Time (s)'); end
        end
    end
end

% This function just adds supertitles to each figure (future max dont forget to remove
% this comment)
function sgtitle_figs(fig_nums, title_str)
    for fi = 1:length(fig_nums)
        figure(fig_nums(fi));
       
        set(gcf, 'Name', title_str, 'NumberTitle', 'on');
        

        try

            sgtitle({title_str, ' ', ' '}, 'FontWeight', 'bold');
        catch

        end
    end
end

figHandles = findall(0, 'Type', 'figure'); 

for i = 1:numel(figHandles) 
    % Extract fig num
    figNum = figHandles(i).Number; 
    
    % Forces file name
    fileName = sprintf('Figure_%d.png', figNum);
    
    saveas(figHandles(i), fileName); 
end
