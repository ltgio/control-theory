%  Tested on:
%  Matlab 2014a using casADi-MatlabR2014a-v.3.0.0-rc3
%  Matlab 2014b using casADi-MatlabR2014b-v.3.0.0-rc3

clc;clear all;close all;
import casadi.*                  % Load CasADi                 
%% Declare variables
x   = SX.sym('x',3);
f   = x(1)^2 + 100*x(3)^2;
g   = x(3) + (1-x(1))^2 - x(2);

x0  = [2.5;3.0;0.75];            % initial guess

% Create IPOPT solver object
% http://www.coin-or.org/Ipopt/documentation/node40.html
opts                             = struct;
opts.ipopt.max_iter              = 20;
opts.ipopt.linear_solver         = 'ma57';
opts.ipopt.hessian_approximation = 'limited-memory';
%opts.ipopt.hessian_approximation = 'exact';

nlp     = struct('x', x, 'f', f, 'g', g);

solver  = nlpsol('solver', 'ipopt', nlp,opts);

w0  = x0;   % initial guess array
lbw = -inf; % lower bound solution
ubw =  inf; % upper bound solution
lbg = 0;    % lower bound inequality array
ubg = 0;    % upper bound inequality array

% Solve the NLP
sol    = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);
%sol    = solver('x0', w0);


%% Solve the NLP and print solution
f_opt     = full(sol.f)           % >> 0
x_opt     = full(sol.x)           % >> [0; 1; 0]
lam_x_opt = full(sol.lam_x)       % >> [0; 0; 0]
lam_g_opt = full(sol.lam_g)       % >> 0

