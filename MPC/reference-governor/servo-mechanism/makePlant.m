function [Plant,Hy,Hc] = makePlant
%% ServoMechanism model
% Parameters
betaM = 0.1;
kt    = 10;
kteta = 1280.2;
betaL = 25;
JM    = 0.5;
R     = 20;
p     = 20;
JL    = 20*JM;

% Model
A = [0,1,0,0;
    -kteta/JL,-betaL/JL,kteta/(p*JL),0;
     0,0,0,1;
     kteta/(p*JM),0,-kteta/(p^2*JM),-(betaM+kt^2/R)/JM];
B  = [0;0;0;kt/(R*JM)];
C  = [1,0,0,0;kteta,0,-kteta/p,0];% [thetaLoad|Torque]
D  = [0;0];

Plant = ss(A,B,C,D,...
'Name'     ,'Servo Mechanism',...
'StateName',{'thetaL','omegaL','thetaM','omegaM'},...
'StateUnit',{'[rad]','[rad/s]','[rad]','[rad/s]'},...
'InputName',{'V'},...
'InputUnit',{'[Volt]'},...
'OutputName',{'thelaL','Torque'},...
'OutputUnit',{'[rad]','[N]'});

[nx,nu] = size(Plant.B); 
[np,nm] = size(Plant.D);

s   = tf('s');
%% PID
% Gcs = [278.2*(s+5.564)*(s+0.3055)]/[s*(s+4.509)];
% [Ac,Bc,Cc,Dc] = ssdata(Gcs);
% Gcs = ss(Ac,Bc,Cc,Dc,...
% 'Name'     ,'PID',...
% 'StateName',{'xPid1','xPid2'},...
% 'InputName',{'thetaL'},...
% 'InputUnit',{'[rad]'},...
% 'OutputName',{'V'},...
% 'OutputUnit',{'[Volt]'});

%% Loop-shaping Controller
%sisotool(Plant(1))
Gcs =  [84.823*(s+2.453)*(s+0.1109)]/[s*(s+1.637)];
[Ac,Bc,Cc,Dc] = ssdata(Gcs);
Gcs = ss(Ac,Bc,Cc,Dc,...
'Name'     ,'PID good',...
'StateName',{'xPid1','xPid2'},...
'InputName',{'thetaL'},...
'InputUnit',{'[rad]'},...
'OutputName',{'V'},...
'OutputUnit',{'[Volt]'});

%% Precompensated Plant 
Plant = series(Gcs,Plant,[1],[1]);
Plant = feedback(Plant,1,[1],[1]);
H     = [Plant.C;zeros(length(1),nx),Cc]; 
Hy    = H(1,:);  % [thetaL]
Hc    = H(2:end,:);% [Torque; V]

% Toss = 10;
T_reference = 12;             % Reference Time
ts = 0.05;
N_reference = T_reference/ts; 
time        = linspace(0, T_reference, N_reference)';
reference   = zeros(N_reference,1); 
reference(N_reference/6:2*N_reference/3) = pi/2;
lsim(Plant,reference,time);
end


