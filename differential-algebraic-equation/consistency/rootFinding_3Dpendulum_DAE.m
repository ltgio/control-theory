% Title: Find initial Condition for pendulum 3D [DAE]
clc;clear all;close all;
import casadi.*                  % Load CasADi       

%% Declare variables
L  = 1;  % lenght pendulum

x   = SX.sym('x',6); % states
p   = x(1:3);        % position
dp  = x(4:6);        % velocity

%% specify invariance for Pendulum 
% min 0
%  s.t. lbg < g(x) < ubg

c  = 0.5*(p'*p - L^2);  % c(t)
dc = p'*dp;             % dc(t)
g  = [c;dc];            % equality constaint 

%% Specify initial guess 
p0  = [L;0.1;-0.45];
dp0 = [0.2;1;0.5];
x0  = [p0;dp0];            % initial guess

%% Create IPOPT solver object
% http://www.coin-or.org/Ipopt/documentation/node40.html
opts                =  struct;
opts.ipopt.linear_solver         = 'ma86';
%opts.ipopt.hessian_approximation = 'limited-memory';
opts.ipopt.hessian_approximation = 'exact';

nlp     = struct('x', x, 'f', 0, 'g', g);
solver  = nlpsol('solver','ipopt', nlp,opts);

% Solve the NLP
sol    = solver('x0', x0, 'lbx', -inf, 'ubx', inf,'lbg', 0, 'ubg', 0);

%% Solve the NLP and print solution
f_opt     = full(sol.f)     % >> 0
x_opt     = full(sol.x)     % >> [0.9037;0.0624;-0.4235;0.1715;0.9973;0.5129]
lam_x_opt = full(sol.lam_x) % >> [0; 0; 0]
lam_g_opt = full(sol.lam_g) % >> 0

