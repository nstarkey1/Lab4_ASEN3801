
function PlotAircraftSim(time, aircraft_state_array, fig, col)

figure(fig(1));
title("inertial position")
subplot(311)
plot(time, aircraft_state_array(1,:), col); hold on;
ylabel('x_e');
subplot(312);
plot(time, aircraft_state_array(2,:),Col); hold on;
ylabel('y_e');
subplot(313)
plot(time, aircraft_state_array(3,:),Col); hold on;
ylabel('z_e');
xlabel('Time (s)')

figure(fig(2));
title("Eueler Angle")
plot(time, aircraft_state_array(4,:), col); hold on;
ylabel('roll (rad)')
subplot(312);
plot(time, aircraft_state_array(5,:),Col); hold on;
ylabel('pitch (rad)')
subplot(313)
plot(time, aircraft_state_array(6,:),Col); hold on;
ylabel('yaw (rad)')
xlabel('Time (s)')

figure(fig(3));
title("velocity")
plot(time, aircraft_state_array(7,:), col); hold on;
ylabel('u (m/s)')
subplot(312);
plot(time, aircraft_state_array(8,:),Col); hold on;
ylabel('v (m/s)')
subplot(313)
plot(time, aircraft_state_array(9,:),Col); hold on;
ylabel('w (m/s)')
xlabel('Time (s)')

figure(fig(4));
title("Angular rate")
plot(time, aircraft_state_array(10,:), col); hold on;
ylabel('p (rad/s)');
subplot(312);
plot(time, aircraft_state_array(11,:),Col); hold on;
ylabel('q (rad/s)');
subplot(313)
plot(time, aircraft_state_array(12,:),Col); hold on;
ylabel('r (rad/s)');
xlabel('Time (s)')

figure(5)
plot3(aircraft_state_array(1,:),aircraft_state_array(2,:), aircraft_state_array(3,:))
xlabel('x'); ylabel('y');zlabel('z');
end
