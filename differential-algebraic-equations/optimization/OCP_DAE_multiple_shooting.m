%%      This file is part of CasADi.
% 
%      CasADi -- A symbolic framework for dynamic optimization.
%      Copyright (C) 2010-2014 Joel Andersson, Joris Gillis, Moritz Diehl,
%                              K.U. Leuven. All rights reserved.
%      Copyright (C) 2011-2014 Greg Horn
% 
%      CasADi is free software; you can redistribute it and/or
%      modify it under the terms of the GNU Lesser General Public
%      License as published by the Free Software Foundation; either
%      version 3 of the License, or (at your option) any later version.
% 
%      CasADi is distributed in the hope that it will be useful,
%      but WITHOUT ANY WARRANTY; without even the implied warranty of
%      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
%      Lesser General Public License for more details.
% 
%      You should have received a copy of the GNU Lesser General Public
%      License along with CasADi; if not, write to the Free Software
%      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
% 
% 

import casadi.*

% Solves the following optimal control problem (OCP) in differential-algebraic
% equations (DAE)

% minimize     integral_{t=0}^{10} x0^2 + x1^2 + u^2  dt
% x0,x1,z,u
% 
% subject to   dot(x0) == z*x0-x1+u     \
%              dot(x1) == x0             }  for 0 <= t <= 10
%                    0 == x1^2 + z - 1  /
%              x0(t=0) == 0
%              x1(t=0) == 1
%              x0(t=10) == 0
%              x1(t=10) == 0
%              -0.75 <= u <= 1  for 0 <= t <= 10

% The method used is direct multiple shooting.

% Joel Andersson, 2015

% Declare variables
x0 = SX.sym('x0',1);
x1 = SX.sym('x1',1);
x  = [x0;x1];         % Differential states
z  = SX.sym('z',1);   % Algebraic variable
u  = SX.sym('u',1);   % Control Input

nx = length(x);
nu = length(u);
nz = length(z);

f_x = [z*x0-x1+u; x0];   % Differential equation
f_z = x1^2 + z - 1;      % Algebraic equation
f_q = x0^2 + x1^2 + u^2; % Lagrange cost term (quadrature)

dae  = struct('x',x, 'z',z, 'p',u, 'ode',f_x, 'alg',f_z,'quad',f_q);
opts = struct('tf',0.5); % interval length
I    = casadi.integrator('I','idas',dae,opts);

% Number of intervals
N = 20;

% Start with an empty NLP
w   = {};  % List of variables
lbw = [];  % Lower bounds on w
ubw = [];  % Upper bounds on w
g   = {};  % Constraints
J   = 0;   % Cost function

% Initial conditions
x0  = [0;1]; 
Xk  = MX.sym('X0', nx);
w   = {w{:}, Xk};  % w.append(Xk)
lbw = [lbw; x0];   % lbw += [ 0, 1 ]
ubw = [ubw; x0];   % ubw += [ 0, 1 ]

x_min = [-inf;-inf];
x_max = [ inf; inf];

u_min = [-0.75];
u_max = [ 1.00];

% # Loop over all intervals
for k = 0:N-1  % for k in range(nk):
    % New NLP variable for the control
    Uk  = MX.sym(['U_' num2str(k)],nu); % Uk = MX.sym('U'+str(k))
    w   = {w{:}, Uk};                   % w.append(Uk)
    % bound on input   
    lbw = [lbw;  u_min];                % lbw += [-0.75]
    ubw = [ubw;  u_max];                % ubw += [ 1.00]
    % Call integrator function
    Ik  = I('x0',Xk,'p',Uk);            % Ik = I(x0=Xk, p=Uk)
    Xk  = Ik.xf;                        % Xk = Ik['xf']
    J   = J + Ik.qf;                    % J = J + Ik['qf'] # Sum quadratures
    % "Lift" the variable
    X_prev = Xk;                        % X_prev = Xk
    Xk  = MX.sym(['X_' num2str(k)],nx); % Xk = MX.sym('X'+str(k+1), 2)
    w   = {w{:}, Xk};                   % w.append(Xk)
    lbw = [lbw; x_min];                 % lbw += [-inf, -inf]
    ubw = [ubw; x_max];                 % ubw += [ inf,  inf]
    g   = {g{:}, X_prev - Xk};          % G.append(X_prev - Xk)   
end

% Allocate an NLP solver
nlp  = struct('x', vertcat(w{:}),'f', J, 'g', vertcat(g{:}));
% option IPOPT
% see: http://www.coin-or.org/Ipopt/documentation/node39.html
opts                             = struct;
opts.ipopt.max_iter              = 200;
opts.ipopt.linear_solver         = 'ma27';
opts.ipopt.hessian_approximation = 'exact';

% solver = nlpsol('solver', 'ipopt', nlp, opts)
solver = nlpsol('solver', 'ipopt', nlp, opts);

%% Solve the NLP

sol    = solver('x0', 0, 'lbx', lbw, 'ubx', ubw,'lbg', 0, 'ubg', 0);
x_sol  = full(sol.x);

time   = linspace(0, 10, N+1)';
x1_opt = x_sol(1:nx+nu:end);
x2_opt = x_sol(2:nx+nu:end);
u_opt  = x_sol(3:nx+nu:end);

% Plot the results
figure;hold on;grid on;
title('Van der Pol optimization - multiple shooting')
plot(time, x1_opt,'r--');
plot(time, x2_opt,'b-');
stairs(time, [u_opt;nan],'k-.');
xlabel('time');
legend(['x0 trajectory','x1 trajectory','u trajectory'])
