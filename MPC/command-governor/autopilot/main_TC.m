%% Reset workspace
clc;
clear;
close all;

%% Preliminary
load('longitudinal_inputRate.mat');

%% Timing

Ts   = 0.01;     % sample time

% Get system dimension
[nx, nu]    = size(B);

% Select output states
C = eye(nx);
C(4:end,:) = []; % remove [Q; da; dT]
C(1:2,:)   = []; % remove [Vt; alpha]

[np,~]  = size(C); 

sysTC   = ss(A,B,eye(nx),zeros(nx,nu));

%% LQI Control for TC system 

% Build augmented system
A_aug = [A  , zeros(nx,np); 
         C  ,  zeros(np,np)];      
B_aug = [B; zeros(np,nu)];
C_aug = [zeros(np,nx) eye(np)];
D_aug = zeros(np,nu);

Q = diag([0.1, 10, 1, 0.1, 1, 1, 1000]); 
R = diag([10,10]);

K = lqr(A_aug, B_aug, Q, R);
K_fb = K(:,1:nx);
K_ff = K(:,nx+1:end);

intEffect     = tf(1,[1 0]);
intSys        = ss(K_ff*intEffect);

feedbackSys    = feedback(sysTC,K_fb);
seriesSys     = series(intSys, feedbackSys);
C             = [0 0 1 0 0 0]; 
closedLoop    = feedback(seriesSys,C);
[Acl,Bcl,~,~] = ssdata(closedLoop);

%% Simulate at Tc 

T_sim         = 20;  

% Define reference for traking
ref           = 10; %[deg]

sim('simTC')

% Plotting
time = sim_eulerAngles(:,1);

figure(1)
plot(time, rad2deg(sim_eulerAngles(1:end,2)),'k','Linewidth',2)
hold on;
plot(time, rad2deg(sim_reference(1:end,2)),'r-.','Linewidth',1.5)
grid on;
legend('\phi(t)','r(t)')
title('Pitch attitude')
ylabel('$\phi(t)\mathring{}$','interpreter', 'latex')
xlabel('time')
disp('Figure 1 >> Pitch attitude')

figure(2)
subplot(2,1,1)
plot(time, sim_aeroState(1:end,2),'Color',[0.85 0.33 0.1],'Linewidth',2)
grid on;
title('Aero States')
ylabel('$V_t(t)$','interpreter','latex')
xlabel('time')
subplot(2,1,2)
plot(time, rad2deg(sim_aeroState(1:end,3)),'Color',[0.93 0.65 0.12],'Linewidth',2)
grid on;
ylabel('$\alpha(t)$','interpreter','latex')
xlabel('time')
disp('Figure 2 >> Aero States: Airspeed and angle of attack')

figure(3)
subplot(2,1,1)
plot(time(1:end-2), rad2deg(sim_realInput(:,2)),'Linewidth',2)
grid on;
title('Input')
ylabel('Elevator')
xlabel('time')
subplot(2,1,2)
plot(time(1:end-2), rad2deg(sim_realInput(:,3)),'r','Linewidth',2)
grid on;
ylabel('Thrust')
xlabel('time')
disp('Figure 3 >> Real input')

figure(4)
subplot(2,1,1)
plot(time, rad2deg(sim_Inputs(1:end,2)),'Color',[0.49 0.18 0.56],'Linewidth',2)
grid on;
title('Derivative input')
ylabel('dEle','interpreter','latex')
xlabel('time')
subplot(2,1,2)
plot(time, rad2deg(sim_Inputs(1:end,3)),'Color',[0 0.5 0],'Linewidth',2)
grid on;
ylabel('dTht','interpreter','latex')
xlabel('time')
disp('Figure 4 >> Derivative input')

disp('Press any key to close all figures')
pause()
close all;
