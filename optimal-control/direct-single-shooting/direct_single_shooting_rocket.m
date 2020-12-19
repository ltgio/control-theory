clc;clear all;close all;

import casadi.*

nx = 3;
nu = 1;
T  = 10;
% Control
u = MX.sym('u',1);
% State
x = MX.sym('x',3);
s = x(1); % position
v = x(2); % speed
m = x(3); % mass

% ODE right hand side
sdot = v;
vdot = (u - 0.05 * v*v)/m;
mdot = -0.1*u^2;
xdot = [sdot;vdot;mdot];

%ODE right hand side function
f = Function('f',{x,u},{xdot});

% Integrate with Explicit Euler over 0.2 seconds
N  = 20;    % number of steps
dt = 0.01;  % Time step
xj = x;

for j=1:N
  fj = f(xj,u);
  xj = xj + dt*fj;
end

% Discrete time dynamics function
F = Function('F', {x,u},{xj});

% Number of control segments
N = 50; 

% Control for all segments
U = MX.sym('U',N); 
 
% Initial conditions
x0 = [0;0;1];
X0 = MX(x0);

% Integrate over all intervals
X = X0;
for k=1:N
  X = F(X,U(k));
end

% Objective function and constraints
J = U'*U;

G = X(1:nx); % final state 

% NLP
nlp = struct('x',U, 'f',J, 'g',G);
 
% Allocate an NLP solver
opts              = struct;
opts.ipopt.tol    = 1e-10;

solver = nlpsol('solver','ipopt', nlp, opts);

w0  = 0.4;             % initial guess array
lbw = -0.5;            % lower bound solution
ubw =  0.5;            % upper bound solution
lbg = [1;0;-inf];      % lower bound inequality array
ubg = [1;0;+inf];      % upper bound inequality array

% Solve the NLP
sol    = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);

%% Solve the NLP and print solution
f_opt     = full(sol.f);          % 
w_opt     = full(sol.x);          % 
lam_x_opt = full(sol.lam_x);      % 
lam_g_opt = full(sol.lam_g);      % 

% Plot the solution
u_opt = w_opt;
x_opt = x0;
for k=0:N-1
    Fk    = F(x_opt(:,end),u_opt(k+1));
    x_opt = [x_opt, full(Fk)];
end
x1_opt = x_opt(1,:);
x2_opt = x_opt(2,:);
x3_opt = x_opt(3,:);
tgrid  = linspace(0, T, N+1);
clf;
hold on
plot(tgrid, x1_opt, 'LineWidth',2);
plot(tgrid, x2_opt, 'LineWidth',2);
plot(tgrid, x3_opt, 'LineWidth',2);
legend('pos','vel','mass');
xlabel('t');grid on;

figure;stairs(tgrid, [u_opt; nan],'LineWidth',2);
xlabel('t');grid on;
