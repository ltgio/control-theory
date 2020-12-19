addpath(genpath('C:\Users\Gianni\Google Drive\SupportCodeNew\Automatic Controls\Gain Scheduling PID'))
% To be Done!!!

clc;clear all;close all;
import casadi.*

%% Model: Magnetic Levitation System
[Fode,ode,y,x,u,nx,nu] = MagneticLevitation;
np = 1;          % number of output to track

%% create integrator for Plant
ts = 0.01; % sample time [s]
M  = 4;    % order integrator
integratorPlant = getIntegrator(Fode,x,u,ts,M);

%% Find Equilibrium Point [ball 1 meter above the reference]
position = 2;
w    = [x;u]; 
w0   = [position; 0; 0;0];
wmin = [position;-inf;-inf;-inf];
wmax = [position; inf; inf; inf];

[xeq,ueq] = trim(ode,w,w0,wmin,wmax,nx);
yeq = xeq(1);

trimPoint     = struct;
trimPoint.xeq = xeq;
trimPoint.yeq = yeq;
trimPoint.ueq = ueq;


%% Linearize system
[A,B,C,D,infoR,infoO] = linearize(ode,y,x,u,xeq,ueq);

sys = ss(A,B,C,D,...
     'name'     ,['Mag Lev trimmed p = ',num2str(position),' [m]'],...
     'StateName',{'p','v','i'},...
     'StateUnit',{'[m]','[m/s]','[A]'},...
     'InputName',{'V'},...
     'InputUnit',{'Volt'},...
     'OutputName',{'p'},...
     'OutputUnit',{'[m]'},...
     'UserData',trimPoint,...
     'Notes', [infoR,' | ',infoO]); 

%% Build augmented state for [PID case] 
s = tf('s');
Gc = [-54573*(s+3.886)*(s+0.1376)]/[s*(s+160)*(s+81.82)];
          
[Ac,Bc,Cc,Dc] = ssdata(Gc);
Gcs = ss(Ac,Bc,Cc,Dc,...
'Name'     ,'PID',...
'StateName',{'xc1','xc2','xc3'},...
'InputName',{'p'},...
'InputUnit',{'[m]'},...
'OutputName',{'V'},...
'OutputUnit',{'[Volt]'});

%% Build Precompensated Plant [PID case]
[Phi,G,H,L] = makePrecompensatedPlant(sys,Gcs);

SystemCompensated = ss(Phi,G,H,L,...
     'name'     ,'Mag Lev + PID',...
     'StateName',{'p','v','i','xc1','xc2','xc3'},...
     'StateUnit',{'[m]','[m/s]','[A]','[m]','[m/s]','[m/s]'},...
     'InputName',{'p'},...
     'InputUnit',{'[p]'},...
     'OutputName',{'p','u'},...
     'OutputUnit',{'[m]','[V]'},...
     'UserData',trimPoint,...
     'Notes', [infoR,' | ',infoO]);
 
%% Create struct for simulink
GainScheduling.parameter = position;
GainScheduling.ueq = ueq;
GainScheduling.yeq = yeq;
GainScheduling.Ac  = Ac;
GainScheduling.Bc  = Bc;
GainScheduling.Cc  = Cc;
GainScheduling.Dc  = Dc;

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
 
%% open Simulink
%linearSystemAnalyzer(sysCL)
T   = 20;        % [s]
open('magLev_sim')
