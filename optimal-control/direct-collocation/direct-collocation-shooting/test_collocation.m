% Title:   check collocation integrator
% Version: Matlab 2014a/casadi3.0.0-rc3
% Author:  Marie Curie PhD student Giovanni Licitra
% Data:    07-03-2016

import casadi.*
% Declare model variables
x1   = MX.sym('x1');
x2   = MX.sym('x2');
x    = [x1; x2];
u    = MX.sym('u');
% Model equations
xdot = [(1-x2^2)*x1 - x2 + u; x1];
% Objective term
L    = x1^2 + x2^2 + u^2;
% Time discretization
T = 10; % Time horizon
N = 20; % number of control intervals
M = 4; % IRK steps per interval
% Create discrete time dynamics
dae = struct('x', x, 'p', u, 'ode', xdot, 'quad', L);
F = collocation(dae, T/N, M);
% Evaluate at a test point
[X_test, Q_test] = F([0.2, 0.3], 0.4);
display(X_test); % DM([0.335539, 0.434784])
display(Q_test); % DM(0.183287)
