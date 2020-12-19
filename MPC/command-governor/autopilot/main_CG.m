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

T_sim   = 20;
N_steps = T_sim/Ts;

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
time = 0:Ts:T_sim-Ts;

figure(1)
plot(time, rad2deg(x_final(3,:)),'k','Linewidth',2)
hold on;
plot(time, rad2deg(ref_cg*ones(1,N_steps)),'r-.','Linewidth',1.5)
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
plot(time, 0.1.*ones(1,N_steps),'r--','Linewidth',2)
hold on;
plot(time, -0.1.*ones(1,N_steps),'r--','Linewidth',2)
grid on;
ylabel('$\omega_x$','interpreter','latex')
xlabel('time')
title('Constraints')

subplot(2,1,2)
plot(time, rad2deg(x_con(2,:)),'Linewidth',2)
hold on;
plot(time, rad2deg(dde_max).*ones(1,N_steps),'r--','Linewidth',2)
hold on;
plot(time, -dde_max.*ones(1,N_steps),'r--','Linewidth',2)
grid on;
ylabel('$dEle$','interpreter','latex')
xlabel('time')
disp('Figure 3 >> Constraints')

figure(4)
stem(t_steps(1:end))
hold on;
plot(0:1:N_steps-1, 0.01.*ones(N_steps),'r--','Linewidth',2)
title('Execution time')
ylabel('$T_c$','interpreter','latex')
disp('Figure 4 >> Time of excecution')

disp(' ')
disp('Press any key to close all figures')
disp(' ')

%%=========================================================================
