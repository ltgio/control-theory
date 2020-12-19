%% Reset workspace
clc;
% clear all;
close all;

%% Preliminary
load('Linearization23ms.mat');

A23;   % A matrix at 23 m/s
B23;   % B matrix at 23 m/s
xs23;  % state eq at 23 m/s
us23 ; % input eq at 23 m/s

eig(A23);

%% Timing

Ts   = 0.01;     % sample time
Toss = 40 ;      % simulation time

%% Building dynamic model
% State vector:
% x = [pN; pE; pD; Vt; alpha; beta; theta; phi; psi; P; Q; R]
% New state vector after selection: [Vt; alpha; phi; Q]
% Select longitudinal dynamic

A          = A23; 
A(12,:)    = []; A(:,12)    = []; % remove R
A(9:10,:)  = []; A(:,9:10)  = []; % remove psi P
A(6:7,:)   = []; A(:,6:7)   = []; % remove beta theta
A(1:3,:)   = []; A(:,1:3)   = []; % remove positionNED 

% Select input signals
% u = [de; da; dr; df; dT];

B          = B23;
B(12,:)    = []; % remove row of the correnspondentig state
B(8:10,:)  = []; %                  ||
B(6,:)     = []; %                  ||
B(1:3,:)   = []; %                  ||

B(:,4)     = []; % remove flaps contribution
B(:,3)     = []; % remove rudder contribution
B(:,2)     = []; % remove ailerons contribution

% Get system dimension
[nx, nu]    = size(B);

%% Introduce the derivatives of input as controls

A = [A, B; zeros(nu, nu+nx)];
B = [zeros(nx, nu); eye(nu)];

% Get dimension of new system
[nx, nu]    = size(B);

if(rank(ctrb(A,B)) == nx)
    disp('System is completely reachble');
else
    disp('System is not completely reachble');
end
clc
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

feedbackSys   = feedback(sysTC,K_fb);
seriesSys     = series(intSys, feedbackSys);
C             = [0 0 1 0 0 0]; 
closedLoop    = feedback(seriesSys,C);
[Acl,Bcl,~,~] = ssdata(closedLoop);

%% Simulate at Tc 

disp(' __________________________________________________________ ')
disp('/ -------------------------------------------------------- \')
disp('| Simulation of the continuos time system with LQI control |')
disp('| -------------------------------------------------------- |')
disp('|__________________________________________________________|')

T_sim         = 20;  

% Define reference for traking
ref           = 10; %[deg]

sim('simTC')

% Plotting
time = sim_eulerAngles(:,1);

disp(' ')
disp('Plotting results:')
disp('Press any key to continue')
disp(' ')
pause()

figure(1)
plot(time, rad2deg(sim_eulerAngles(:,2)),'k','Linewidth',2)
hold on;
plot(time, rad2deg(sim_reference(:,2)),'r-.','Linewidth',1.5)
grid on;
legend('\phi(t)','r(t)')
title('Pitch attitude')
ylabel('$\phi(t)\mathring{}$','interpreter', 'latex')
xlabel('time')
disp('Figure 1 >> Pitch attitude')

figure(2)
subplot(2,1,1)
plot(time, sim_aeroState(:,2),'Color',[0.85 0.33 0.1],'Linewidth',2)
grid on;
title('Aero States')
ylabel('$V_t(t)$','interpreter','latex')
xlabel('time')
subplot(2,1,2)
plot(time, rad2deg(sim_aeroState(:,3)),'Color',[0.93 0.65 0.12],'Linewidth',2)
grid on;
ylabel('$\alpha(t)$','interpreter','latex')
xlabel('time')
disp('Figure 2 >> Aero States: Airspeed and angle of attack')

figure(3)
subplot(2,1,1)
plot(time(1:end), rad2deg(sim_realInput(:,2)),'Linewidth',2)
grid on;
title('Input')
ylabel('Elevator')
xlabel('time')
subplot(2,1,2)
plot(time(1:end), rad2deg(sim_realInput(:,3)),'r','Linewidth',2)
grid on;
ylabel('Thrust')
xlabel('time')
disp('Figure 3 >> Real input')

figure(4)
subplot(2,1,1)
plot(time, rad2deg(sim_Inputs(:,2)),'Color',[0.49 0.18 0.56],'Linewidth',2)
grid on;
title('Derivative input')
ylabel('dEle','interpreter','latex')
xlabel('time')
subplot(2,1,2)
plot(time, rad2deg(sim_Inputs(:,3)),'Color',[0 0.5 0],'Linewidth',2)
grid on;
ylabel('dTht','interpreter','latex')
xlabel('time')
disp('Figure 4 >> Derivative input')

disp(' ')
disp('Press any key to close all figures and continue with simulation')
disp(' ')

pause();
clc
close all;

%% ========================================================================

clearvars -except A B sysTC Ts nx nu np T_sim ref

disp(' __________________________________________________________ ')
disp('/ -------------------------------------------------------- \')
disp('| Simulation of the discrete time system with DLQI control |')
disp('| -------------------------------------------------------- |')
disp('|__________________________________________________________|')

%% Discrete version
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

% Simulate at TD 

sim('simTD')

% Plotting
time = sim_eulerAnglesTD(:,1);

disp(' ')
disp('Plotting results:')
disp('Press any key to continue')
disp(' ')
pause()

% figure(1)
% plot(time, rad2deg(sim_eulerAnglesTD(1:end,2)),'k','Linewidth',2)
% hold on;
% plot(time, rad2deg(sim_reference(1:end-2,2)),'r-.','Linewidth',1.5)
% grid on;
% legend('\phi(t)','r(t)')
% title('Pitch attitude')
% ylabel('$\phi(t)\mathring{}$','inter
% preter', 'latex')
% xlabel('time')
% disp('Figure 1 >> Pitch attitude')

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
plot(time, rad2deg(sim_realInputTD(:,2)),'Linewidth',2)
grid on;
title('Input')
ylabel('Elevator')
xlabel('time')
subplot(2,1,2)
plot(time, rad2deg(sim_realInputTD(:,3)),'r','Linewidth',2)
grid on;
ylabel('Thrust')
xlabel('time')
disp('Figure 3 >> Real input')

figure(4)
subplot(2,1,1)
plot(time, rad2deg(sim_InputsTD(:,2)),'Color',[0.49 0.18 0.56],'Linewidth',2)
grid on;
title('Derivative input')
ylabel('dEle','interpreter','latex')
xlabel('time')
subplot(2,1,2)
plot(time, rad2deg(sim_InputsTD(:,3)),'Color',[0 0.5 0],'Linewidth',2)
grid on;
ylabel('dTht','interpreter','latex')
xlabel('time')
disp('Figure 4 >> Derivative input')

disp(' ')
disp('Press any key to close all figures and continue with simulation')
disp(' ')

pause();
clc
close all;

%% Command governor

disp(' __________________________________________________________ ')
disp('/ -------------------------------------------------------- \')
disp('|                      COMMAND GOVERNOR                    |')
disp('| -------------------------------------------------------- |')
disp('|__________________________________________________________|')
% disp(' ')
% disp('Starting simulation')
% disp(' ')
% disp('Press any key to start')
% pause

%% define Command Governor

clearvars -except Kd_fb Cdi closedLoopTD Ts nx nu np T_sim

Phi = closedLoopTD.a;
G   = closedLoopTD.b;
Hy  = closedLoopTD.c;
Hd  = closedLoopTD.d;

Hc  = [0 0 0 1 0 0 0; -Kd_fb Cdi];
Hc  = Hc(1:2,:);
ng  = size(G,2);
nv  = size(Hc,1);     

L   = zeros(nv,ng);

T   = zeros(2*nv,nv);
row_s = 1;
row_e = 2;
  for j=1:nv
    for i=row_s:row_e
        T(i,j) = (-1)^(i+1);
    end
  row_s = row_s+2;
  row_e = row_e+2;
  end

clear row_s row_e;  

%% Define constraints

dde_max = 2;%3.25;  % [rad/s] actuatorDynamics.Value.elevator.rateLimit
dda_max = 2.50;  % [rad/s] actuatorDynamics.Value.aileron.rateLimit
ddr_max = 4.00;  % [rad/s] actuatorDynamics.Value.rudder.rateLimit

b = [0.1; 0.1; dde_max; dde_max];

% Tolleranza algoritmo CG
delta = 1e-3;
 
k0 = 100;

Psi = eye(ng);

[A_Wd, b_Wd, Ax_Vx, AW_Vx, b_Vx] = CG_Offline(Phi,G,Hc,L,T,b,delta,k0);


%% MatLab Simulation

N_steps = 1000;
x0      = zeros(7,1);

x_final = zeros(7,N_steps);
x_con   = zeros(nv,N_steps);
t_steps = zeros(1,N_steps);
cg      = zeros(1,N_steps);

% Initialization
x_curr      = x0;
load('reference');

for i = 1:N_steps
    tic
    g            = CG(reference(i), x_curr, Psi, A_Wd, b_Wd, Ax_Vx, AW_Vx, b_Vx);  
    t_steps(i)   = toc;
    x_next       = Phi*x_curr + G*g;
    x_curr       = x_next;
    x_con(:,i)   = Hc*x_curr;
    x_final(:,i) = x_curr;
    cg(i)        = g;
    per = i/N_steps*100;
    sper = num2str(per);
    clc
    perc = strcat('Completed: ',sper,'%');
    disp(perc)
end

disp(' ')
disp('Plotting results:')
disp('Press any key to continue')
disp(' ')
pause()

% Plotting
time = 0:Ts:10-Ts;

figure(1)
plot(time, rad2deg(x_final(3,:)),'k','Linewidth',2)
hold on;
plot(time, rad2deg(reference*ones(1,1000)),'r-.','Linewidth',1.5)
grid on;
legend('\phi(t)','r(t)')
title('Pitch attitude')
ylabel('$\phi(t)\mathring{}$','interpreter', 'latex')
xlabel('time')
disp('Figure 1 >> Pitch attitude')

figure(2)
subplot(2,1,1)
plot(time, x_final(1,:),'Color',[0.85 0.33 0.1],'Linewidth',2)
grid on;
title('Aero States')
ylabel('$V_t(t)$','interpreter','latex')
xlabel('time')
subplot(2,1,2)
plot(time,  x_final(2,:),'Color',[0.93 0.65 0.12],'Linewidth',2)
grid on;
ylabel('$\alpha(t)$','interpreter','latex')
xlabel('time')
disp('Figure 2 >> Aero States: Airspeed and angle of attack')

figure(3)
subplot(2,1,1)
plot(time, x_con(1,:),'Linewidth',2)
hold on;
plot(time, 0.1.*ones(1,1000),'r--','Linewidth',2)
hold on;
plot(time, -0.1.*ones(1,1000),'r--','Linewidth',2)
grid on;
ylabel('$\omega_x$','interpreter','latex')
xlabel('time')
title('Constraints')

subplot(2,1,2)
plot(time, x_con(2,:),'Linewidth',2)
hold on;
plot(time,dde_max.*ones(1,1000),'r--','Linewidth',2)
hold on;
plot(time, -dde_max.*ones(1,1000),'r--','Linewidth',2)
grid on;
ylabel('$dEle$','interpreter','latex')
xlabel('time')
disp('Figure 3 >> Constraints')

figure(4)
stem(t_steps(1:end))
hold on;
plot(0.01:0.01:1000, 0.01.*ones(1,100000),'r--','Linewidth',2)
title('Execution time')
ylabel('$T_c$','interpreter','latex')
disp('Figure 4 >> Time of excecution')

disp(' ')
disp('Press any key to close all figures')
disp(' ')

%%=========================================================================