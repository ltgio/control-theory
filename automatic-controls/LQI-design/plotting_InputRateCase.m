time = sim_aeroState(:,1);

%% aerostate
figure(1)
subplot(3,1,1)
plot(time,sim_aeroState(:,2),'LineWidth',2)
title('Airspeed');
xlabel('Time');
ylabel('$V_t(t)$','interpreter','latex');
grid on;

subplot(3,1,2)
plot(time,rad2deg(sim_aeroState(:,3)),'LineWidth',2);
title('Angle of attack');
xlabel('Time');
ylabel('$\alpha(t)$ [deg]','interpreter','latex');
grid on;

subplot(3,1,3)
plot(time,rad2deg(sim_aeroState(:,4)),'LineWidth',2)
title('Sideslip angle');
xlabel('Time');
ylabel('$\beta(t)$ [deg]','interpreter','latex');
grid on;

% ========================================================================
%% euler angles
figure(2)
subplot(3,1,1);hold on;
plot(time,rad2deg(sim_eulerAngles(:,2)),'LineWidth',2);
plot(time,rad2deg(sim_reference(:,2)),'LineWidth',1,'Color','r');
legend('y(t) roll','r(t) roll');
title('roll angle');
xlabel('Time');
ylabel('$\theta(t)$ [deg]','interpreter','latex');
grid on;

subplot(3,1,2);hold on;
plot(time,rad2deg(sim_eulerAngles(:,3)),'LineWidth',2)
plot(time,rad2deg(sim_reference(:,3)),'LineWidth',1,'Color','r');
legend('y(t) pitch','r(t) pitch');
title('pitch angle');
xlabel('Time');
ylabel('$\phi(t)$ [deg]','interpreter','latex');
grid on;

subplot(3,1,3);hold on;
plot(time,rad2deg(sim_eulerAngles(:,4)),'LineWidth',2)
legend('y(t) yaw');
title('yaw angle');
xlabel('Time');
ylabel('$\psi(t)$ [deg]','interpreter','latex');
grid on;

% ========================================================================
% angularRate
figure(3)
subplot(3,1,1)
plot(time,sim_angularRate(:,2),'LineWidth',2)
title('Angular velocity x-axis');
xlabel('Time');
ylabel('$R(t)$ [rad/s]','interpreter','latex');
grid on;

subplot(3,1,2)
plot(time,sim_angularRate(:,3),'LineWidth',2)
title('Angular velocity y-axis');
xlabel('Time');
ylabel('$Q(t)$ [rad/s]','interpreter','latex');
grid on;

subplot(3,1,3)
plot(time,sim_angularRate(:,4),'LineWidth',2)
title('Angular velocity z-axis');
xlabel('Time');
ylabel('$R(t)$  [rad/s]','interpreter','latex');
grid on;


% ========================================================================
figure(4)
subplot(2,2,1)
plot(time,rad2deg(sim_ControlDeflection(:,2)),'LineWidth',2)
title('Elevator');
xlabel('Time');
ylabel('$de(t)$ [deg]','interpreter','latex');
grid on;

subplot(2,2,2)
plot(time,rad2deg(sim_ControlDeflection(:,3)),'LineWidth',2)
title('Ailerons');
xlabel('Time');
ylabel('$da(t)$ [deg]','interpreter','latex');
grid on;

subplot(2,2,3)
plot(time,rad2deg(sim_ControlDeflection(:,4)),'LineWidth',2)
title('Rudder');
xlabel('Time');
ylabel('$dr(t)$ [deg]','interpreter','latex');
grid on;

subplot(2,2,4)
plot(time,sim_ControlDeflection(:,5),'LineWidth',2)
title('Thrust');
xlabel('Time');
ylabel('$dT(t)$','interpreter','latex');
grid on;

% ========================================================================
figure(5)
subplot(2,2,1)
plot(time,rad2deg(sim_InputsRate(:,2)),'LineWidth',2)
title('dElevator');
xlabel('Time');
ylabel('$dde(t)$ [deg]','interpreter','latex');
grid on;

subplot(2,2,2)
plot(time,rad2deg(sim_InputsRate(:,3)),'LineWidth',2)
title('dAilerons');
xlabel('Time');
ylabel('$dda(t)$ [deg]','interpreter','latex');
grid on;

subplot(2,2,3)
plot(time,rad2deg(sim_InputsRate(:,4)),'LineWidth',2)
title('dRudder');
xlabel('Time');
ylabel('$ddr(t)$ [deg]','interpreter','latex');
grid on;

subplot(2,2,4)
plot(time,sim_InputsRate(:,5),'LineWidth',2)
title('dThrust');
xlabel('Time');
ylabel('$ddT(t)$','interpreter','latex');
grid on;