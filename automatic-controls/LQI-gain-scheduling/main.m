addpath(genpath('C:\Users\Gianni\Google Drive\SupportCode\Automatic Controls\Gain Scheduling LQI'))

clc;clear all;close all;

import casadi.*

%% Model: Magnetic Levitation System
[Fode,dx,y,x,u,nx,nu] = MagneticLevitation;
np = 1;          % number of output to track

%% create integrator for Plant
ts = 0.01; % sample time [s]
M  = 4;    % order integrator
integratorPlant = getIntegrator(Fode,x,u,ts,M);

%% Find Equilibrium Point [ball 1 meter above the reference]
position = 1;
w    = [x;u]; 
w0   = [position; 0; 0;0];
wmin = [position;-inf;-inf;-inf];
wmax = [position; inf; inf; inf];

[xeq,ueq] = trim(dx,w,w0,wmin,wmax,nx);

trimPoint     = struct;
trimPoint.xeq = xeq;
trimPoint.ueq = ueq;

%% Linearize system
%[A,B,C,D,infoR,infoO] = linearize(ode,y,x,u,xeq,ueq);
A_fun = Function('A',{x,u},{jacobian(dx,x)},char('x','u'), char('A'));
B_fun = Function('B',{x,u},{jacobian(dx,u)},char('x','u'), char('B'));
C_fun = Function('C',{x,u},{jacobian(y,x)} ,char('x','u'), char('C'));
D_fun = Function('D',{x,u},{jacobian(y,u)} ,char('x','u'), char('D'));

A     = full(A_fun(xeq,ueq));
B     = full(B_fun(xeq,ueq));
C     = full(C_fun(xeq,ueq));
D     = full(D_fun(xeq,ueq));

sys = ss(A,B,C,D,...
     'name'     ,['Mag Lev trimmed p = ',num2str(position),' [m]'],...
     'StateName',{'p','v','i'},...
     'StateUnit',{'[m]','[m/s]','[A]'},...
     'InputName',{'V'},...
     'InputUnit',{'Volt'},...
     'OutputName',{'p'},...
     'OutputUnit',{'[m]'}); 

%      'UserData',trimPoint,...
%      'Notes', [infoR,' | ',infoO]
%% Build augmented state for LQI
Q = diag([1,0.1,1,1]);  % [p;v;i;ep]
R = 1;                  % [V]
[Kfb,Kff] = LQIdesign(sys,Q,R,nx,nu);

%% Build Precompensated Plant [LQI case]
[Phi,G,H,L] = makePrecompensatedPlant(sys,Kfb,Kff);

SystemCompensated = ss(Phi,G,H,L,...
     'name'     ,'Mag Lev + LQI',...
     'StateName',{'p','v','i','ep'},...
     'StateUnit',{'[m]','[m/s]','[A]','[m]'},...
     'InputName',{'p'},...
     'InputUnit',{'[p]'},...
     'OutputName',{'p','u'},...
     'OutputUnit',{'[m]','[V]'});

%% Create struct for simulink
GainScheduling.parameter = position;
GainScheduling.Kfb = Kfb;
GainScheduling.Kff = Kff;
GainScheduling.xeq = xeq;
GainScheduling.ueq = ueq;

% Use the function Simulink.Bus.createObject to create Simulink.Bus 
% objects that represent the structure 
Simulink.Bus.createObject(GainScheduling)
% The bus is named slBus1. Rename it.
GainSchedulingStruct = slBus1;
clear slBus1
% Store the structure myStruct in a Simulink.Parameter object.
GainScheduling = Simulink.Parameter(GainScheduling);
% Set the data type of the parameter object to the bus object myParamsType.
GainScheduling.DataType = 'Bus: GainSchedulingStruct';

%% Build Precompensated non linear plant
[FodeLQI,odeLQI,Output,xa,r,na,nr] = MagneticLevitationClosedLoop(Kff,Kfb,xeq,ueq);
integratorPlantCloseLoop = getIntegrator(FodeLQI,xa,r,ts,M);%% run simulation without simulink

%% forward simulation NL model + LQI
N    = 500;
T    = ts*N;        % [s]
time = linspace(0,T,N);

Xsim = zeros(na,N);
Ysim = zeros(2,N-1);
reference = 2; % reference

Xsim(:,1) = [xeq;0]; % initial condition

for i=1:N-1  
  Xnext       = full(integratorPlantCloseLoop(Xsim(:,i),reference));
  Ysim(:,i)   = full(Output(Xsim(:,i)));
  Xsim(:,i+1) = Xnext;
end

position = Ysim(1,:);
control  = Ysim(2,:);

%% plot
figure;
subplot(5,1,1);hold on;grid on;
plot(time,Xsim(1,:),'Color','b','LineWidth',2);
plot(time, reference.*ones(length(time),1),'k-.');
ylabel('p [m]');
legend('p [m]','ref');
xlim([0,time(end)]);

subplot(5,1,2);hold on;grid on;
plot(time,Xsim(2,:),'Color','b','LineWidth',2);
ylabel('v [m/s]');
xlim([0,time(end)]);

subplot(5,1,3);hold on;grid on;
plot(time,Xsim(3,:),'Color','b','LineWidth',2);
ylabel('i [A]');
xlim([0,time(end)]);

subplot(5,1,4);hold on;grid on;
plot(time,Xsim(4,:),'Color','b','LineWidth',2);
ylabel('x_{e} [m]');
xlim([0,time(end)]);

subplot(5,1,5);hold on;grid on;
stairs(time(1:end-1),control,'Color','r','LineWidth',2);
ylabel('u [V]');
xlim([0,time(end)]);

%% open Simulink
%linearSystemAnalyzer(sysCL)
%open('magLev_sim')


