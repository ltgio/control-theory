%% Example 4.4-2: Pitch-SAS Design
%  Author: Giovanni Licitra
%  Aircraft Control And Simulation: 293

clc;clear all;close all;

%%  create model 
A = [-1.9311*10^-2  ,  8.8157      , -3.2170*10^1 , -5.7499*10^-1;
     -2.5389*10^-4  , -1.0189      ,  0           ,  9.0506*10^-1;
      0             ,  0           ,  0           ,  1           ;
      2.9465*10^-12 ,  8.2225*10^-1,  0           , -1.0774     ];

B = [ 1.7370*10^-1;
     -2.1499*10^-3;
      0           ;
     -1.7555*10^-1]; 
 
C = [0 , 57.29578 , 0 ,  0       ; 
     0 ,  0       , 0 , 57.29578];
 
tau_a = 1/20.2; % constant time actuator
tau_f = 0.1;    % constant time alpha noise filter

[nx,nu] = size(B);
np = 2;

Aaug = [A,-B,zeros(nx,nu);
        zeros(nu,nx), -1/tau_a , zeros(nu,nu);
        0 1/tau_f 0 0 0 -1/tau_f];
Baug = [zeros(nx,nu);1/tau_a;0];
Caug = [C,zeros(np,np);
        zeros(1,4), 0 57.29578];
    
LongDyn_F16 = ss(Aaug,Baug,Caug,zeros(3,1),...
                    'Name'     ,'Longitudinal Dynamics F16',...
                    'StateName',{'v','alpha','theta','q','x_actuator','x_filterAoA'},...
                    'StateUnit',{'[m/s]','[rad]','[rad]','[rad/s]','[rad]','[rad]'},...
                    'InputName',{'de'},...
                    'InputUnit',{'[rad]'},...
                    'OutputName',{'alpha','q','alpha filtered'},...
                    'OutputUnit',{'[deg]','[deg/s]','[deg]'});

Ka  = 0.1; % gain alpha feedback chosen 
Acl = LongDyn_F16.A - LongDyn_F16.B*Ka*LongDyn_F16.C(3,:);  % close the loop with ka
qfb = ss(Acl,LongDyn_F16.B,LongDyn_F16.C(2,:),0);           % SISO system for q feedback

%%
z = 3; p = 1;
lag  = ss(-p,1,z-p,1);  % lag compensator
csys = series(lag,qfb); % cascade Compensator before plant

[a,b,c,d] = ssdata(csys);

k = logspace(-2,0,2000);
rlocus(a,b,c,d,k);grid on
