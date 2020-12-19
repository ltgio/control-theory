%% Simple LP proble
% qpoases using exact hessian
clc;clear all;close all;
import casadi.* 
% Declare variables

x = SX.sym('x',3,1);

f = x(1)^2 + 2*x(2) - x(3);
g = x(1) + x(2) + x(3);

%% Create IPOPT solver object
nlp     = struct('x',x, 'f', f, 'g', g);
opts    = struct;

% Pick an NLP solver
%mySolver = 'ipopt';
%mySolver = 'worhp'; % plugin not installed
mySolver = 'sqpmethod';

if strcmp(mySolver,'ipopt')
  opts.ipopt.max_iter = 5000;
end

if strcmp(mySolver,'sqpmethod')
  opts.qpsol = 'qpoases';
  opts.qpsol_options.printLevel = 'none';
end

% Allocate a solver
solver  = nlpsol('solver',mySolver,nlp,opts);

w0  = [0   ; 0   ; 0  ];  % initial guess array
lbw = [0   ; 0   ; 0  ];  % lower bound solution
ubw = [inf ; inf ; inf];  % upper bound solution
lbg = 1;                  % lower bound inequality array
ubg = 1;                  % upper bound inequality array

%% Solve the NLP and print solution
sol     = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);
%sol    = solver('x0', w0);

f_opt     = full(sol.f)           % >> 50
x_opt     = full(sol.x)           % >> [5;5]
lam_x_opt = full(sol.lam_x)       % >> [0;0]
lam_g_opt = full(sol.lam_g)       % >> -10
