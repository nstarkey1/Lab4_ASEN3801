function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
%% Fig 1 Inertial Pos
figure(fig(1))

subplot(3,1,1)
plot(time, aircraft_state_array(1,:), col, 'LineWidth', 1.5); hold on;
grid on
ylabel('x_E (m)')
title('Inertial Position vs Time')

subplot(3,1,2)
plot(time, aircraft_state_array(2,:), col, 'LineWidth', 1.5); hold on;
grid on
ylabel('y_E (m)')

subplot(3,1,3)
plot(time, aircraft_state_array(3,:), col, 'LineWidth', 1.5); hold on;
grid on
ylabel('z_E (m)')
xlabel('Time (s)')


%% Fig 2 Euler Angs
figure(fig(2))

subplot(3,1,1)
plot(time, rad2deg(aircraft_state_array(4,:)), col, 'LineWidth', 1.5); hold on;
grid on
ylabel('\phi (deg)')
title('Euler Angles vs Time')

subplot(3,1,2)
plot(time, rad2deg(aircraft_state_array(5,:)), col, 'LineWidth', 1.5); hold on;
grid on
ylabel('\theta (deg)')

subplot(3,1,3)
plot(time, rad2deg(aircraft_state_array(6,:)), col, 'LineWidth', 1.5); hold on;
grid on
ylabel('\psi (deg)')
xlabel('Time (s)')


%% Fig 3 body frame translatioal vels
figure(fig(3))

subplot(3,1,1)
plot(time, aircraft_state_array(7,:), col, 'LineWidth', 1.5); hold on;
grid on
ylabel('u (m/s)')
title('Body Velocities vs Time')

subplot(3,1,2)
plot(time, aircraft_state_array(8,:), col, 'LineWidth', 1.5); hold on;
grid on
ylabel('v (m/s)')

subplot(3,1,3)
plot(time, aircraft_state_array(9,:), col, 'LineWidth', 1.5); hold on;
grid on
ylabel('w (m/s)')
xlabel('Time (s)')


%% Fig 4 body frame ang rates
figure(fig(4))

subplot(3,1,1)
plot(time, aircraft_state_array(10,:), col, 'LineWidth', 1.5); hold on;
grid on
ylabel('p (rad/s)')
title('Angular Rates vs Time')

subplot(3,1,2)
plot(time, aircraft_state_array(11,:), col, 'LineWidth', 1.5); hold on;
grid on
ylabel('q (rad/s)')

subplot(3,1,3)
plot(time, aircraft_state_array(12,:), col, 'LineWidth', 1.5); hold on;
grid on
ylabel('r (rad/s)')
xlabel('Time (s)')


%% Fig 5 control inputs
figure(fig(5))

subplot(4,1,1)
plot(time, control_input_array(1,:), col, 'LineWidth', 1.5); hold on;
grid on
ylabel('Z_c (N)')
title('Control Inputs vs Time')

subplot(4,1,2)
plot(time, control_input_array(2,:), col, 'LineWidth', 1.5); hold on;
grid on
ylabel('L_c (N{\cdot}m)')

subplot(4,1,3)
plot(time, control_input_array(3,:), col, 'LineWidth', 1.5); hold on;
grid on
ylabel('M_c (N{\cdot}m)')

subplot(4,1,4)
plot(time, control_input_array(4,:), col, 'LineWidth', 1.5); hold on;
grid on
ylabel('N_c (N{\cdot}m)')
xlabel('Time (s)')


%% Fig 6 3d flight path
figure(fig(6))

plot3(aircraft_state_array(1,:), ...
      aircraft_state_array(2,:), ...
     -aircraft_state_array(3,:), ...
      col, 'LineWidth', 1.5);
hold on
grid on
axis equal

xlabel('x_E (m)')
ylabel('y_E (m)')
zlabel('Height (m)')
title('Quadrotor 3D Flight Path')

% Start and End markers

plot3(aircraft_state_array(1,1), ...
      aircraft_state_array(2,1), ...
     -aircraft_state_array(3,1), ...
      'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8)

 
plot3(aircraft_state_array(1,end), ...
      aircraft_state_array(2,end), ...
     -aircraft_state_array(3,end), ...
      'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8)

end
