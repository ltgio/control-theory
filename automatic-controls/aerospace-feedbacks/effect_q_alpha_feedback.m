%% Example 4.4-1: Effect of Pitch Rate and Alpha Feedback
%  Author: Giovanni Licitra
%  Aircraft Control And Simulation: 289

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

%% Perform rootlocus using alpha filtered feedback 
k  = logspace(-2,1,2000);
figure;rlocus(LongDyn_F16.A,LongDyn_F16.B,LongDyn_F16.C(3,:),0,k)
grid on;hold on;

%% Perform rootlocus using alpha q feedback 
Ka = 0.5; % gain alpha feedback chosen 
Acl = LongDyn_F16.A - LongDyn_F16.B*Ka*LongDyn_F16.C(3,:);  % close the loop with ka
%[z,p,k] = ss2zp(Acl,LongDyn_F16.B,LongDyn_F16.C(2,:),0);   % q/de transfer function
%de2q_tf = zpk(z,p,k)
figure;rlocus(Acl,LongDyn_F16.B,LongDyn_F16.C(2,:),0);grid on;

%% 
Kq  = 0.25;
Acl = Acl - LongDyn_F16.B*Kq*LongDyn_F16.C(2,:)

LongDyn_F16_SAS = ss(Acl,Baug,Caug,zeros(3,1),...
                    'Name'     ,'Longitudinal Dynamics F16',...
                    'StateName',{'v','alpha','theta','q','x_actuator','x_filterAoA'},...
                    'StateUnit',{'[m/s]','[rad]','[rad]','[rad/s]','[rad]','[rad]'},...
                    'InputName',{'de'},...
                    'InputUnit',{'[rad]'},...
                    'OutputName',{'alpha','q','alpha filtered'},...
                    'OutputUnit',{'[rad]','[rad/s]','[rad]'});

figure;impulse(LongDyn_F16_SAS)               

%% Alteranite 
G2 = ss(A,B,C,zeros(2,1));
Ga = ss(-1/tau_a,1/tau_a,1,0);
Gf = ss(-1/tau_f,1/tau_f,1,0);

Ga2  = series(Ga,G2); 
Ga2f = series(Ga2,Gf,1,1);

%sys = series(sys1,sys2,outputs1,inputs2) 
