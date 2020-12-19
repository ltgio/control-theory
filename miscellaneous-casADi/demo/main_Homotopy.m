%% title: Homotopy strategy
clc;clear all;close all;
import casadi.*                  % Load CasADi                 
%% Declare variables
x      = SX.sym('x',3);
lambda = SX.sym('lambda');

f   = 10*x(1)^2 + x(3)^2;
g   = x(3) + lambda*((1-x(1))^2);

% NB: in this case we start with a linear constraint and we go to a non
% linear one. lambda start from 0 -> 1

x0  = [2.5;3.0;0.75];            % initial guess

% Create IPOPT solver object
% http://www.coin-or.org/Ipopt/documentation/node40.html
opts                             = struct;
opts.ipopt.max_iter              = 20;
opts.ipopt.linear_solver         = 'ma57';
opts.ipopt.hessian_approximation = 'limited-memory';
opts.ipopt.hessian_approximation = 'exact';

nlp     = struct('x', x, 'p',lambda, 'f', f, 'g', g);

solver  = nlpsol('solver', 'ipopt', nlp,opts);

w0  = x0;   % initial guess array
lbw = -inf; % lower bound solution
ubw =  inf; % upper bound solution
lbg = 0;    % lower bound inequality array
ubg = 0;    % upper bound inequality array

figure(1)
hold on
for p=linspace(0,1,10)
  p
  % Solve the NLP
  sol   = solver('x0', w0, 'p', p, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);
  w0    = sol.x;
  x_opt = full(sol.x)
  plot(full(x_opt(1)),full(x_opt(2)),'o');
end


