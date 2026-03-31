function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col, legend_str, line_style)

if nargin < 6
    legend_str = 'Sim Data';
end
if nargin < 7
    line_style = '-';
end

%% Fig 1 Inertial Pos
figure(fig(1))
subplot(3,1,1)
plot(time, aircraft_state_array(1,:), 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str); hold on;
grid on
ylabel('x_E (m)')
title('Inertial Position vs Time')
legend('Location', 'best')

subplot(3,1,2)
plot(time, aircraft_state_array(2,:), 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str); hold on;
grid on
ylabel('y_E (m)')
legend('Location', 'best')

subplot(3,1,3)
plot(time, aircraft_state_array(3,:), 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str); hold on;
grid on
ylabel('z_E (m)')
xlabel('Time (s)')
legend('Location', 'best')

%% Fig 2 Euler Angs
figure(fig(2))
subplot(3,1,1)
plot(time, rad2deg(aircraft_state_array(4,:)), 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str); hold on;
grid on
ylabel('\phi (deg)')
title('Euler Angles vs Time')
legend('Location', 'best')

subplot(3,1,2)
plot(time, rad2deg(aircraft_state_array(5,:)), 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str); hold on;
grid on
ylabel('\theta (deg)')
legend('Location', 'best')

subplot(3,1,3)
plot(time, rad2deg(aircraft_state_array(6,:)), 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str); hold on;
grid on
ylabel('\psi (deg)')
xlabel('Time (s)')
legend('Location', 'best')

%% Fig 3 Body Frame Translational Vels
figure(fig(3))
subplot(3,1,1)
plot(time, aircraft_state_array(7,:), 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str); hold on;
grid on
ylabel('u (m/s)')
title('Body Velocities vs Time')
legend('Location', 'best')

subplot(3,1,2)
plot(time, aircraft_state_array(8,:), 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str); hold on;
grid on
ylabel('v (m/s)')
legend('Location', 'best')

subplot(3,1,3)
plot(time, aircraft_state_array(9,:), 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str); hold on;
grid on
ylabel('w (m/s)')
xlabel('Time (s)')
legend('Location', 'best')

%% Fig 4 Body Frame Ang Rates
figure(fig(4))
subplot(3,1,1)
plot(time, aircraft_state_array(10,:), 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str); hold on;
grid on
ylabel('p (rad/s)')
title('Angular Rates vs Time')
legend('Location', 'best')

subplot(3,1,2)
plot(time, aircraft_state_array(11,:), 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str); hold on;
grid on
ylabel('q (rad/s)')
legend('Location', 'best')

subplot(3,1,3)
plot(time, aircraft_state_array(12,:), 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str); hold on;
grid on
ylabel('r (rad/s)')
xlabel('Time (s)')
legend('Location', 'best')

%% Fig 5 Control Inputs
figure(fig(5))
subplot(4,1,1)
plot(time, control_input_array(1,:), 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str); hold on;
grid on
ylabel('Z_c (N)')
title('Control Inputs vs Time')
legend('Location', 'best')

subplot(4,1,2)
plot(time, control_input_array(2,:), 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str); hold on;
grid on
ylabel('L_c (N{\cdot}m)')
legend('Location', 'best')

subplot(4,1,3)
plot(time, control_input_array(3,:), 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str); hold on;
grid on
ylabel('M_c (N{\cdot}m)')
legend('Location', 'best')

subplot(4,1,4)
plot(time, control_input_array(4,:), 'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str); hold on;
grid on
ylabel('N_c (N{\cdot}m)')
xlabel('Time (s)')
legend('Location', 'best')

%%% Fig 6 3D Flight Path
figure(fig(6))
plot3(aircraft_state_array(1,:), ...
      aircraft_state_array(2,:), ...
     -aircraft_state_array(3,:), ...
      'Color', col, 'LineStyle', line_style, 'LineWidth', 1.5, 'DisplayName', legend_str);
hold on
grid on
axis equal
xlabel('Inertial x Position (m)')
ylabel('Inertial y Position (m)')
zlabel('Inertial Z Position (m)')
title('3D Position of Quadrotor')
legend('Location', 'best')

% Start marker (green dot)
plot3(aircraft_state_array(1,1), ...
      aircraft_state_array(2,1), ...
     -aircraft_state_array(3,1), ...
      'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'HandleVisibility', 'off')

% End marker (red asterisk)
plot3(aircraft_state_array(1,end), ...
      aircraft_state_array(2,end), ...
     -aircraft_state_array(3,end), ...
      'r*', 'MarkerSize', 8, 'HandleVisibility', 'off')
end
