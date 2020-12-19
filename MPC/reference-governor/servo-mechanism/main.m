%%%%%%% This script loads needed variable in the workspace and starts the simulation.
close all;clear all;clc;
global nx nr

%% Precompensated Plant
% xdot(t) = PHI*x(t) + G*r(t)
%    y(t) = Hy*x(t) -> thetaL
%    c(t) = Hc*x(t) -> (torque [N];voltage [V])
[Plant,Hy,Hc] = makePlant;
PHI     = Plant.A;
G       = Plant.B;
[nx,nr] = size(G);

%% Set Reference Governor
Th = 1;    % Time horizon [s]
ts = 0.05; % Sample time [s]

Tbound  = 78.5398;         % Torque Bound [N]
Vbound  = 220;             % Voltage Bound [Volt]
Cset    = [Tbound;Vbound]; % Set constraint

solverOpt = 'ipopt';
%solverOpt = 'qpoases';

[QPsolver,lbg,ubg,N] = makeReferenceGovernor(G,PHI,Hy,Hc,Cset,Th,ts,solverOpt);

%% Create trajectory for tracking
T_reference = 12;             % Reference Time
N_reference = T_reference/ts; 
reference   = zeros(N_reference,nr); 
reference(N_reference/6:2*N_reference/3) = pi/2;

%% Initialize Plot
time     = linspace(0, T_reference, N_reference+1)';
time_opt = time(1:N);

figure('units','normalized','outerposition',[0 0 1 1])
subplot(4,1,1);hold on;grid on;
traj_ref     = plot(time,[reference;nan] ,'LineWidth',1,'Color','b');
traj_ref_opt = stairs(time_opt,zeros(N,1),'LineWidth',1,'Color','r');
traj_y_opt   = stairs(time_opt,zeros(N,1),'LineWidth',1,'Color','g');
legend('reference','predictive reference','output');
axis([0, time(end), -1 ,3]);
subplot(4,1,2);hold on;grid on;
traj_torque  = stairs(time_opt,zeros(N,1),'LineWidth',1,'Color','b','LineStyle','-');
plot(time  ,-Tbound.*ones(N_reference+1,1),'LineWidth' ,1,'Color','k','LineStyle','-.');
plot(time  , Tbound.*ones(N_reference+1,1),'LineWidth' ,1,'Color','k','LineStyle','-.');
axis([0, time(end), -100 ,100]);
legend('Torque [N]');
subplot(4,1,3);hold on;grid on;
traj_voltage = stairs(time_opt,zeros(N,1),'LineWidth',1,'Color','b','LineStyle','-');
plot(time  ,-Vbound.*ones(N_reference+1,1),'LineWidth' ,1,'Color','k','LineStyle','-.');
plot(time  ,+Vbound.*ones(N_reference+1,1),'LineWidth' ,1,'Color','k','LineStyle','-.');
legend('Voltage [V]');
axis([0, time(end), -250 ,250]);
subplot(4,1,4);hold on;grid on;
traj_CPU = stem(0,0,'mo');
axis([0, time(end) , 0 , 0.1]);
legend('CPU time');
xlabel('time [s]');

waitforbuttonpress

plot_ref     = [];
plot_thetaL  = [];
plot_torque  = [];
plot_voltage = [];
plot_CPU     = [];

%%
ref_N = reference(1:N); % reference to be tracked
x0p   = zeros(nx,1);    % current state
p     = [x0p;ref_N];    % parametric vector to be feed to the QP

for i = 1:N_reference-N
    tic;
    sol      = QPsolver('lbg', lbg, 'ubg', ubg,'p',p); % solveQp
    
    CPU_time = toc;
    
    w_opt  = full(sol.x);                            % get solution from QP
    r_opt  = w_opt(1:nx+nr:end);                     % get optimized reference
    x1_opt = [x0p(1);w_opt(2:nx+nr:end)];            
    x2_opt = [x0p(2);w_opt(3:nx+nr:end)];
    x3_opt = [x0p(3);w_opt(4:nx+nr:end)];
    x4_opt = [x0p(4);w_opt(5:nx+nr:end)];
    x5_opt = [x0p(5);w_opt(6:nx+nr:end)];
    x6_opt = [x0p(6);w_opt(7:nx+nr:end)];
    
    x_opt      = [x1_opt,x2_opt,x3_opt,x4_opt,x5_opt,x6_opt];
    y_thetaL   = Hy*x_opt';  
    c_opt      = Hc*x_opt';
    y_torque   = c_opt(1,:);
    y_Voltage  = c_opt(2,:);
    
    plot_ref     = [plot_ref    ;r_opt(1)    ];
    plot_thetaL  = [plot_thetaL ;y_thetaL(1) ];
    plot_torque  = [plot_torque ;y_torque(1) ];
    plot_voltage = [plot_voltage;y_Voltage(1)];
    plot_CPU     = [plot_CPU    ;CPU_time    ];
    
    time_opt  = time_opt + ts; 
    set(traj_ref_opt, 'XData', time_opt);
    set(traj_ref_opt, 'YData', r_opt);
    set(traj_y_opt  , 'XData', time_opt);
    set(traj_y_opt  , 'YData', y_thetaL(1:end-1));
    
    set(traj_torque , 'XData', time_opt);
    set(traj_torque , 'YData', y_torque(1:end-1));
    set(traj_voltage, 'XData', time_opt);
    set(traj_voltage, 'YData', y_Voltage(1:end-1));
    
    set(traj_CPU    , 'XData', time_opt(1));
    set(traj_CPU    , 'YData', CPU_time);
    
    %waitforbuttonpress
    pause(ts)
    drawnow
    
    ref_N = reference(i:(N-1)+i); % reference to be tracked
    x0p   = x_opt(2,:)';    % current state
    p     = [x0p;ref_N];    % parametric vector to be feed to the QP
end

plot_time = linspace(0,length(plot_ref)*ts,length(plot_ref))';

figure('units','normalized','outerposition',[0 0 1 1]);
subplot(4,1,1);hold on;grid on;
stairs(plot_time,reference(1:length(plot_time)) ,'LineWidth',1,'Color','b');
stairs(plot_time,plot_ref   ,'LineWidth',1,'Color','r');
stairs(plot_time,plot_thetaL,'LineWidth',1,'Color','g');
legend('reference','predictive reference','output');
%axis([0, time(end), -1 ,3]);
subplot(4,1,2);hold on;grid on;
stairs(plot_time,plot_torque,'LineWidth',1,'Color','b','LineStyle','-');
plot(plot_time  ,-Tbound.*ones(length(plot_time),1),'LineWidth' ,1,'Color','k','LineStyle','-.');
plot(plot_time  , Tbound.*ones(length(plot_time),1),'LineWidth' ,1,'Color','k','LineStyle','-.');
%axis([0, time(end), -100 ,100]);
legend('Torque [N]');
subplot(4,1,3);hold on;grid on;
stairs(plot_time,plot_voltage,'LineWidth',1,'Color','b','LineStyle','-');
plot(plot_time  ,-Vbound.*ones(length(plot_time),1),'LineWidth' ,1,'Color','k','LineStyle','-.');
plot(plot_time  ,+Vbound.*ones(length(plot_time),1),'LineWidth' ,1,'Color','k','LineStyle','-.');
legend('Voltage [V]');
%axis([0, time(end), -250 ,250]);
subplot(4,1,4);hold on;grid on;
stem(plot_time,plot_CPU,'mo');
legend('CPU time');
xlabel('time [s]');