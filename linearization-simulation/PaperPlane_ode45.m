function PaperPlane_ode45
% Paper Airplane Modelling - Giovanni licitra - date: 20-Jun-15
% model:
% input:  u = angle of attack [rad]
% states: x = [px;pz;vx;vz] with p = position [m] v = velocity [m/s]
% nx = 4 | nu = 1


clc;close all;
m = 2;           % mass [kg] 
AR = 10;         % Aspect ration
rho = 1.2;       % air density
g = 9.81;        % gravity [m/s^2]
Sref = 0.5;      % area wing [m^2]
p = [m;AR;rho;g;Sref];
x0 = [0;100;5;5];           % [px0;pz0;vx0;vz0] i.c.

% First, define the time-dependent parameters 
T = 10;
ts = 0.01;
t = [0:ts:T-ts]';                               % define range
ut  = t;                                        % time varing parameter
u = 0*pi/180 + 0*sin(2*pi*0.5*t);               % angle of attack [rad]
u(1) = deg2rad(5);                              % 5 deg i.c.

[t,x_ode45] = ode45(@(t,x) ode(t,x,u,ut,p),t, x0);

px_sim = x_ode45(:,1); 
pz_sim = -x_ode45(:,2);
vx_sim = x_ode45(:,3);
vz_sim = x_ode45(:,4);

figure;
subplot(2,3,[1 4]);plot(px_sim,pz_sim,'rx');hold on;grid on;
                   xlabel('p_{x} [m]');ylabel('p_{z} [m]');
subplot(2,3,2);plot(t,vx_sim,'g');hold on;grid on;
               ylabel('v_{x} [m/s]');xlabel('time [s]');
subplot(2,3,3);plot(t,vz_sim,'b');hold on;grid on;
               ylabel('v_{z} [m/s]');xlabel('time [s]');
subplot(2,3,[5 6]);stairs(t,rad2deg(u));hold on;grid on;
               xlabel('time [s]');ylabel('u(t) [deg]');
              
end

function dx = ode(t,x,u,ut,p)                 % Paperplane ODE
u = interp1(ut,u,t);         % Interpolate the data set (ut,u) at time t
px = x(1);pz = x(2);vx = x(3);vz = x(4);            % states
alpha = u;                                          % control 
m = p(1);AR = p(2);rho = p(3);g = p(4);Sref = p(5); % parameters

CL = 2*pi*alpha*(10/12);          
CD = 0.01+CL^2/(AR*pi);
V = norm([vx;vz],2);

eL = 1/V*[ vz;-vx];
eD = 1/V*[-vx;-vz];

Flift = 0.5*rho*V^2*CL*Sref*eL;
Fdrag = 0.5*rho*V^2*CD*Sref*eD;
Fgravity = [0;m*g];
F = Flift+Fdrag+Fgravity;                           
Fx = F(1);Fz = F(2);

dx = [vx;vz;Fx/m;Fz/m];
end

