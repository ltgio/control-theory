%% State
%  x = [positionNED; V   ; alpha   ; beta   ; euler   ;angularVel  ]
% dx = [velocityNED; VDot; alphaDot; betaDot; eulerDot;angularAccel] 
% input  
%  u = [de,da,dr,df,dT]
%  dt = flaps
%  dT = trust

% limit on rate

dde_max = 3.25;  % [rad/s] actuatorDynamics.Value.elevator.rateLimit
dda_max = 2.50;  % [rad/s] actuatorDynamics.Value.aileron.rateLimit
ddr_max = 4.00;  % [rad/s] actuatorDynamics.Value.rudder.rateLimit
%ddth_max = inf;  % for now
    
clc;clear all;close all;
load('Linearization23ms.mat');

A23   % A matrix at 23 m/s
B23   % B matrix at 23 m/s
xs23  % state eq at 23 m/s
us23  % input eq at 23 m/s

disp('eigenvalue at 23 m/s')
eig(A23)

%% Mad
Ts   = 0.01;     % sample time
Toss = 20 ;      % simulation time

A          = A23; 
A(:,[1:3]) = []; % remove positionNED 
A([1:3],:) = []; % from the A matrix

B          = B23;
B(:,4)     = []; % remove flaps contribution
B([1:3],:) = [];

[nx,nu]    = size(B);
%% Introduce the derivatives of input as controls
A = [A,B;zeros(nu,nu+nx)];
B = [zeros(nx,nu);eye(nu)];

% select the which states we want to perform the tracking
% i.e. roll and pitch angle
C = eye(nx+nu);
C(6:end,:) = []; % remove [yaw; p; q; r; de,da,dr,dT]
C(1:3,:)   = []; % remove [V,alpha,beta]

[nx,nu] = size(B); % number of states and input
[np,~]  = size(C); % number of output for tracking

[Ad,Bd,Cd,Dd] = ssdata(c2d(ss(A,B,C,zeros(np,nu)),Ts));

%% Build augmented state for LQI
A_aug = [Ad    , zeros(nx,np); 
         Cd*Ad , eye(np,np)];
       
B_aug = [Bd; Cd*Bd];
C_aug = [zeros(np,nx) eye(np)];
D_aug = zeros(np,nu);

rank(ctrb(A_aug,B_aug))
rank(obsv(A_aug,C_aug))

Qd = diag([0.1 ,10 ,1,...     % V   ; alpha ; beta
           1  ,1 ,1 ,...      % roll; pitch ; yaw
           0.1,0.1,0.1,...    % p   ; q     ; r
           1  ,1  ,1  ,1 ,... % de,da,dr,dT
           10,10]);       % e_roll; e_pitch

%  u = [dde,dda,ddr,ddT]
Rd = diag([1,1,1,1]);
K_aug = dlqr(A_aug, B_aug, Qd, Rd);

K_fb = K_aug(:,1:nx)
K_ff = K_aug(:,nx+1:end)

EffInteg      = tf([1 0],[1 -1],Ts);
sysIntegrale  = ss(K_ff*EffInteg);
[Ai,Bi,Ci,Di] = ssdata(sysIntegrale);


sim('dlqi_inputRate')
plotting_InputRateCase