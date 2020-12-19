% Title:   Direct Multiple Shooting using Collocation integrator for VTOL
% Version: Matlab 2014a/casadi3.0.0-rc3
% Author:  Marie Curie PhD student Giovanni Licitra
% Data:    07-03-2016

clc;clear all;close all;
import casadi.*

%% Time Setting ===========================================================
T      = 5;
N      = 80;              % number of control intervals
M      = 4;               % RK4 steps per interval
DT     = T/N/M;

%% Parameters =============================================================
p       = struct;
p.mass  = 3*10^4;           % massa veivolo [kg]
p.J     = 3*10^4;           % inerzia [kg*m^2]
p.l     = 4;                % lunghezza alare [m]
p.alpha = pi/8;             % angolo [rad]
p.g     = 9.81;             % accelerazione gravitazionale

%% States =================================================================
y      = SX.sym('y'     ,1); 
z      = SX.sym('z'     ,1);
theta  = SX.sym('theta' ,1);
dy     = SX.sym('dy'    ,1);
dz     = SX.sym('dz'    ,1);
dtheta = SX.sym('dtheta',1);

x  = [y;z;theta;dy;dz;dtheta];

Taero  = SX.sym('central_thrust',1);
Faero  = SX.sym('lateral_thrust',1);
u      = [Taero;Faero];

nx     = length(x);
nu     = length(u);

% define ode
xdot = [ dy                                                                    ;...
         dz                                                                    ;...
         dtheta                                                                ;...
        -Taero/p.mass*sin(theta) + 2*Faero/p.mass*sin(p.alpha)*cos(theta)      ;...
         Taero/p.mass*cos(theta) + 2*Faero/p.mass*sin(p.alpha)*sin(theta) - p.g;...
         2*p.l*Faero/p.J*cos(p.alpha)                                             ];

% define Mayer Term
L  = y^2 + z^2 + theta^2 + dy^2 + dz^2 + dtheta^2 +Faero^2 + Taero^2;

% Continuous time dynamics
dae    = struct('x', x, 'p', u, 'ode', xdot, 'quad', L);
% Create discrete time dynamics
F      = collocation(dae, T/N, M);

%% Start with an empty NLP ================================================
w      = {};
w0     = [];
lbw    = [];
ubw    = [];
J      = 0;
g      = {};
lbg    = [];
ubg    = [];

% "Lift" initial conditions
x0     = [1;-1;0.2;0.5;0.3;-0.3];

X0     = MX.sym('X0', nx);
w      = {w{:}, X0};

%% enforce initial condition
lbw    = [lbw;  x0];
ubw    = [ubw;  x0];
w0     = [w0;   x0];

%% Formulate the NLP
Xk      = X0;
for k   = 0:N-1
    % New NLP variable for the control
    Uk  = MX.sym(['U_' num2str(k)],nu);
    w   = {w{:}, Uk};
    
    % bound on input  -inf <= T <= inf [deg] , -10 <= F <= 10 [N] 
    lbw = [lbw;  -inf; -1000];
    ubw = [ubw;   inf;  1000];
    w0  = [w0 ;   zeros(nu,1)];
    
    % Integrate till the end of the interval
    [Xk_end, Jk] = F(Xk, Uk);
    J = J + Jk;
    
    % New NLP variable for state at end of interval
    Xk  = MX.sym(['X_' num2str(k+1)], nx);
    w   = {w{:}, Xk};
    lbw = [lbw;  -inf; -inf; -inf; -inf; -inf; -inf];
    ubw = [ubw;   inf;  inf;  inf;  inf;  inf;  inf];
    w0  = [w0 ;   zeros(nx,1)];
        
    % Add equality constraint
    g   = {g{:}, Xk_end-Xk};
    lbg = [lbg; zeros(nx,1)];
    ubg = [ubg; zeros(nx,1)];
end

% final condition
lbw(end-nx+1:end) = zeros(nx,1);
ubw(end-nx+1:end) = zeros(nx,1);

% Create an NLP solver
prob   = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
% option IPOPT
% see: http://www.coin-or.org/Ipopt/documentation/node39.html
opts                     = struct;
opts.ipopt.max_iter      = 500;
opts.ipopt.linear_solver = 'ma57';

%% Solve the NLP
solver = nlpsol('solver', 'ipopt', prob,opts);
sol    = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);
w_opt  = full(sol.x);

% Plot the solution
y_opt       = w_opt(1:nx+nu:end);
z_opt       = w_opt(2:nx+nu:end);
theta_opt   = w_opt(3:nx+nu:end);
dy_opt      = w_opt(4:nx+nu:end);
dz_opt      = w_opt(5:nx+nu:end);
dtheta_opt  = w_opt(6:nx+nu:end);

T_opt       = w_opt(7:nx+nu:end);
F_opt       = w_opt(8:nx+nu:end);

figure;
time = linspace(0, T, N+1);
subplot(2,2,1);hold on;grid on;
plot(time,y_opt    ,'r');
plot(time,z_opt    ,'b');
plot(time,theta_opt,'g');
legend('y[t]','z[t]','\theta [t]');

subplot(2,2,2);hold on;grid on;
plot(time,dy_opt    ,'r');
plot(time,dz_opt    ,'b');
plot(time,dtheta_opt,'g');
legend('dy[t]','dz[t]','d \theta [t]');

subplot(2,2,3);hold on;grid on;
plot(time,[T_opt;nan],'c');legend('T[t]');
subplot(2,2,4);hold on;grid on;
plot(time,[F_opt;nan],'m');legend('F[t]');

% Inspect Jacobian sparsity
g        = vertcat(g{:});
w        = vertcat(w{:});
Jacobian = jacobian(g, w);
figure(2)
spy(sparse(DM.ones(Jacobian.sparsity())),'r')

% Inspect Hessian of the Lagrangian sparsity
Lambda     = MX.sym('lam', g.sparsity());
Lagrancian = J + dot(Lambda, g);
Hessian    = hessian(Lagrancian, w);
figure(3)
spy(sparse(DM.ones(Hessian.sparsity())),'r')
