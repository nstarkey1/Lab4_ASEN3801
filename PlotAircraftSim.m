function x = PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
%[x,y,z,roll,pit,yaw,u,v,w,p,q,r] = aircraft_state_array;
%[Z,L,M,N] = control_input_array;

% Position Plots
figure(fig(1));
subplot(311);
plot(time, aircraft_state_array(1,:), col); hold on;
subplot(312);
plot(time, aircraft_state_array(2,:), col); hold on;
subplot(313);
plot(time, aircraft_state_array(3,:), col); hold on;


% Euler Angles Plots
figure(fig(2));
subplot(311);
plot(time, aircraft_state_array(4,:), col); hold on;
subplot(312);
plot(time, aircraft_state_array(5,:), col); hold on;
subplot(313);
plot(time, aircraft_state_array(6,:), col); hold on;

% Velocity Plots
figure(fig(3));
subplot(311);
plot(time, aircraft_state_array(7,:), col); hold on;
subplot(312);
plot(time, aircraft_state_array(8,:), col); hold on;
subplot(313);
plot(time, aircraft_state_array(9,:), col); hold on;



% Angular Velocity Plots
figure(fig(4));
subplot(311);
plot(time, aircraft_state_array(10,:), col); hold on;
subplot(312);
plot(time, aircraft_state_array(11,:), col); hold on;
subplot(313);
plot(time, aircraft_state_array(12,:), col); hold on;


% 3D Route Plot
figure(fig(4));
plot3(aircraft_state_array(1,:),aircraft_state_array(2,:),aircraft_state_array(3,:), col); hold on;


end