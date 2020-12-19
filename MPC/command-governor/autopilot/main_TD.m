%% Reset workspace
clc;
clear;
close all;

%% Preliminary
load('longitudinal_inputRate.mat');

% Get system dimension
[nx, nu]    = size(B);
Ts          = 0.01;

% Select output states
C = eye(nx);
C(4:end,:) = []; % remove [Q; da; dT]
C(1:2,:)   = []; % remove [Vt; alpha]

[np,~]  = size(C); 

sysTC   = ss(A,B,eye(nx),zeros(nx,nu));

sysTD         = c2d(sysTC,Ts);
[Ad,Bd,Cd,Dd] = ssdata(sysTD);
Cd            = [0 0 1 0 0 0]; 

%% Build augmented state for LQI
Ad_aug = [Ad    , zeros(nx,np); 
          Cd*Ad ,  eye(np,np)];
       
Bd_aug = [Bd; Cd*Bd];
Cd_aug = [zeros(np,nx) eye(np)];
Dd_aug = zeros(np,nu);

Qd     = diag([0.1, 10, 1, 0.1, 1, 1, 1000]); 
Rd     = diag([10,10]);

Kd     = dlqr(Ad_aug, Bd_aug, Qd, Rd);
Kd_fb  = Kd(:,1:nx);
Kd_ff  = Kd(:,nx+1:end);

intEffectTD       = tf([1 0],[1 -1],Ts);
intSysTD          = ss(Kd_ff*intEffectTD);
[Adi,Bdi,Cdi,Ddi] = ssdata(intSysTD);

feedbackSysTD   = feedback(sysTD,Kd_fb);
seriesSysTD     = series(intSysTD, feedbackSysTD);
Cd              = [0 0 1 0 0 0]; 
closedLoopTD    = feedback(seriesSysTD,Cd);
[Adcl,Bdcl,~,~] = ssdata(closedLoopTD);

%% Simulate at TD 

T_sim = 20;
ref   = 10;
sim('simTD')

% Plotting
time = sim_eulerAnglesTD(:,1);

disp('Plotting results:')

figure(1)
plot(time, rad2deg(sim_eulerAnglesTD(1:end,2)),'k','Linewidth',2)
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
plot(time, sim_aeroStateTD(1:end,2),'Color',[0.85 0.33 0.1],'Linewidth',2)
grid on;
title('Aero States')
ylabel('$V_t(t)$','interpreter','latex')
xlabel('time')
subplot(2,1,2)
plot(time, rad2deg(sim_aeroStateTD(1:end,3)),'Color',[0.93 0.65 0.12],'Linewidth',2)
grid on;
ylabel('$\alpha(t)$','interpreter','latex')
xlabel('time')
disp('Figure 2 >> Aero States: Airspeed and angle of attack')

figure(3)
subplot(2,1,1)
plot(time, rad2deg(sim_realInputTD(1:end,2)),'Linewidth',2)
grid on;
title('Input')
ylabel('Elevator')
xlabel('time')
subplot(2,1,2)
plot(time, rad2deg(sim_realInputTD(1:end,3)),'r','Linewidth',2)
grid on;
ylabel('Thrust')
xlabel('time')
disp('Figure 3 >> Real input')

figure(4)
subplot(2,1,1)
plot(time, rad2deg(sim_InputsTD(1:end,2)),'Color',[0.49 0.18 0.56],'Linewidth',2)
grid on;
title('Derivative input')
ylabel('dEle','interpreter','latex')
xlabel('time')
subplot(2,1,2)
plot(time, rad2deg(sim_InputsTD(1:end,3)),'Color',[0 0.5 0],'Linewidth',2)
grid on;
ylabel('dTht','interpreter','latex')
xlabel('time')
disp('Figure 4 >> Derivative input')

disp('Press any key to close all figures')
pause()
close all;
