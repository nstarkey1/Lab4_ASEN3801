function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
% aircraft_state_array =
% [x,y,z,roll,pitch,yaw,u,v,w,p,q,r]

%% Position Plots
figure(fig(1))

subplot(3,1,1)
plot(time, aircraft_state_array(1,:), col, 'LineWidth',1.5)
grid on
ylabel('x_E (m)')
title('Inertial Position vs Time')

subplot(3,1,2)
plot(time, aircraft_state_array(2,:), col, 'LineWidth',1.5)
grid on
ylabel('y_E (m)')

subplot(3,1,3)
plot(time, aircraft_state_array(3,:), col, 'LineWidth',1.5)
grid on
ylabel('z_E (m)')
xlabel('Time (s)')


%% Euler Angle Plots
figure(fig(2))

subplot(3,1,1)
plot(time, rad2deg(aircraft_state_array(4,:)), col, 'LineWidth',1.5)
grid on
ylabel('\phi (deg)')
title('Euler Angles vs Time')

subplot(3,1,2)
plot(time, rad2deg(aircraft_state_array(5,:)), col, 'LineWidth',1.5)
grid on
ylabel('\theta (deg)')

subplot(3,1,3)
plot(time, rad2deg(aircraft_state_array(6,:)), col, 'LineWidth',1.5)
grid on
ylabel('\psi (deg)')
xlabel('Time (s)')


%% Body Velocity Plots
figure(fig(3))

subplot(3,1,1)
plot(time, aircraft_state_array(7,:), col, 'LineWidth',1.5)
grid on
ylabel('u (m/s)')
title('Body Velocities vs Time')

subplot(3,1,2)
plot(time, aircraft_state_array(8,:), col, 'LineWidth',1.5)
grid on
ylabel('v (m/s)')

subplot(3,1,3)
plot(time, aircraft_state_array(9,:), col, 'LineWidth',1.5)
grid on
ylabel('w (m/s)')
xlabel('Time (s)')


%% Angular Rate Plots
figure(fig(4))

subplot(3,1,1)
plot(time, aircraft_state_array(10,:), col, 'LineWidth',1.5)
grid on
ylabel('p (rad/s)')
title('Angular Rates vs Time')

subplot(3,1,2)
plot(time, aircraft_state_array(11,:), col, 'LineWidth',1.5)
grid on
ylabel('q (rad/s)')

subplot(3,1,3)
plot(time, aircraft_state_array(12,:), col, 'LineWidth',1.5)
grid on
ylabel('r (rad/s)')
xlabel('Time (s)')


%% 3D Route Plot
figure(fig(5))

plot3(aircraft_state_array(1,:), ...
      aircraft_state_array(2,:), ...
     -aircraft_state_array(3,:), ...
      col,'LineWidth',1.5)

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
     -aircraft_state_array(3,1),'go','MarkerFaceColor','g')

plot3(aircraft_state_array(1,end), ...
      aircraft_state_array(2,end), ...
     -aircraft_state_array(3,end),'ro','MarkerFaceColor','r')

end