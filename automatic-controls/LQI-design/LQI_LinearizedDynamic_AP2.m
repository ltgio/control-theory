%% State
%  x = [positionNED; V   ; alpha   ; beta   ; euler   ;angularVel  ]
% dx = [velocityNED; VDot; alphaDot; betaDot; eulerDot;angularAccel] 
% input  
%  u = [de,da,dr,df,dT]
%  dt = flaps
%  dT = trust
clc;clear all;close all;
load('Linearization23ms.mat');

A23   % A matrix at 23 m/s
B23   % B matrix at 23 m/s
xs23  % state eq at 23 m/s
us23  % input eq at 23 m/s

disp('eigenvalue at 23 m/s')
eig(A23)

%% Mad
Ts   = 0.01; % sample time
Toss = 30 ; % simulation time

A          = A23; 
A(:,[1:3]) = []; % remove positionNED 
A([1:3],:) = []; % from the A matrix

B          = B23;
B(:,4)     = []; % remove flaps contribution
B([1:3],:) = [];

% select the which states we want to perform the tracking
C = eye(9);
C(6:9,:) = [];
C(1:3,:) = [];

[nx,nu] = size(B); % number of states and input
[np,~]  = size(C); % number of output for tracking

%% Build augmented state for LQI
A_aug = [A  , zeros(nx,np); 
         C  , eye(np,np)];       
B_aug = [B; zeros(np,nu)];
C_aug = [zeros(np,nx),eye(np)];
D_aug = zeros(np,nu);

Qd = diag([0.1 ,10 ,1,...   % V   ; alpha ; beta
           1  ,1 ,1 ,...    % roll; pitch ; yaw
           0.1,0.1,0.1,...  % p   ; q     ; r
           10,10]);         % e_roll; e_pitch

%  u = [de,da,dr,dT]
Rd = diag([1,1,1,1]);
K_aug = lqr(A_aug, B_aug, Qd, Rd);

K_fb = K_aug(:,1:nx)
K_ff = K_aug(:,nx+1:end)

EffInteg      = tf([1],[1 0]);
sysIntegrale  = ss(K_ff*EffInteg);
[Ai,Bi,Ci,Di] = ssdata(sysIntegrale);

sim('lqi')
plotting
