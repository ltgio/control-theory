addpath(genpath('C:\Users\Gianni\Google Drive\SupportCode\Automatic Controls\Gain Scheduling'))

clc;clear all;close all;
import casadi.*

%% Model: Magnetic Levitation System
[Fode,dx,y,x,u,nx,nu] = MagneticLevitation;
np = 1;        % number of output to track

%% Gain Scheduling
pInit = 1;   % minimum gain position  
pEnd  = 3;   % maximum gain position
Ngain = 3;   % number of gains

GSparameter = linspace(pInit,pEnd,Ngain);             % scheduling is computed in function of position
GSrange     = (GSparameter(2)-GSparameter(1))/2;
GSinterval  = linspace(pInit-GSrange,pEnd+GSrange,Ngain+1);
GSmidpoint  = round(GSparameter(end)/2);
  
nG          = length(GSparameter);  % number of GainScheduling
w           = [x;u];

PlnatCompensated = {nG};
Plant  = {nG};         % Pre-allocation Plants

GS_xeq = zeros(nx,nG); % Pre-allocation equilibrium states 
GS_ueq = zeros(nu,nG); % Pre-allocation equilibrium inputs
GS_Kff = zeros(np,nG); % Pre-allocation feedforward gain  
GS_Kfb = zeros(nx,nG); % Pre-allocation feedback gain    

for i = 1:nG
    % Find Equilibrium Point
    w0     = [GSparameter(i); 0; 0;0];
    wmin   = [GSparameter(i);-inf;-inf;-inf];
    wmax   = [GSparameter(i); inf; inf; inf];
    
    [xeq,ueq] = trim(dx,w,w0,wmin,wmax,nx);

    %% store equilibrium point
    GS_xeq(:,i) = xeq; 
    GS_ueq(:,i) = ueq; 
    
    %% Linearize system
    [A,B,C,D,infoR,infoO] = linearize(dx,y,x,u,xeq,ueq);
   
    sys = ss(A,B,C,D,...
     'name'     ,['Mag Lev trimmed p = ',num2str(i)],...
     'StateName',{'p','v','i'},...
     'StateUnit',{'[m]','[m/s]','[A]'},...
     'InputName',{'V'},...
     'InputUnit',{'Volt'},...
     'Notes'   , [infoR,' | ',infoO]); 
    
    Plant{i} = sys;
    
    %% Get for LQI
    Q = diag([1,0.1,1,1]);  % [p;v;i;ep]
    R = 1;                  % [V]
    [Kfb,Kff] = LQIdesign(sys,Q,R,nx,nu);
    
    GS_Kff(:,i) = Kff;
    GS_Kfb(:,i) = Kfb;
    
    %% Build Precompensated Plant [LQI case]
    [Phi,G,H,L] = makePrecompensatedPlant(sys,Kfb,Kff);

    SystemCompensated = ss(Phi,G,H,L,...
         'name'     ,'Mag Lev + LQI',...
         'StateName',{'p','v','i','ep'},...
         'StateUnit',{'[m]','[m/s]','[A]','[m]'},...
         'InputName',{'p'},...
         'InputUnit',{'[p]'},...
         'OutputName',{'p','u'},...
         'OutputUnit',{'[m]','[V]'},...
         'Notes', [infoR,' | ',infoO]);
     
    PlnatCompensated{i} = SystemCompensated;
end

%% Create struct for simulink
GainScheduling.parameter  = GSparameter;
GainScheduling.GSinterval = GSinterval; 
GainScheduling.GSmidpoint = GSmidpoint;
GainScheduling.Ngain      = Ngain;
GainScheduling.Kfb        = GS_Kfb;
GainScheduling.Kff        = GS_Kff;
GainScheduling.xeq        = GS_xeq;
GainScheduling.ueq        = GS_ueq;

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

%% Bode analysis
% figure;bode(Plant{:});grid on;
% figure;bode(PlnatCompensated{:});grid on;

%% open Simulink
%linearSystemAnalyzer(sysCL)
T    = 10;            % [s]
ts   = 0.01;          % [s]
x0   = [2;0;0.2]; % start from 2 meter height
open('magLev_GS_sim')
