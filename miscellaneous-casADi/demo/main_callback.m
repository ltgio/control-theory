%% title: use the callback on your nlp
clc;clear all;close all;
import casadi.*                  % Load CasADi                 
%% Declare variables
x   = SX.sym('x',3);
p   = SX.sym('p',0,1);           % no parametric variable

f   = x(1)^2 + 100*x(3)^2;
g   = x(3) + (1-x(1))^2 - x(2);

x0  = [2.5;3.0;0.75];            % initial guess
% Use the function

nx = size(x,1); % number of decision variables
ng = size(g,1); % number of constraints
np = size(p,1); % number of parametric variable

Call = MyCallback('f', nx, ng, np);

% Create IPOPT solver object
% http://www.coin-or.org/Ipopt/documentation/node40.html
opts                             = struct;
opts.ipopt.max_iter              = 20;
opts.ipopt.linear_solver         = 'ma57';
%opts.ipopt.hessian_approximation = 'limited-memory';
opts.ipopt.hessian_approximation = 'exact';
opts.iteration_callback          = Call;

nlp    = struct('x', x, 'p', p, 'f', f, 'g', g);
solver = nlpsol('solver', 'ipopt', nlp,opts);

w0  = x0;   % initial guess array
lbw = -inf; % lower bound solution
ubw =  inf; % upper bound solution
lbg = 0;    % lower bound inequality array
ubg = 0;    % upper bound inequality array

iter = 0;
f0   = x0(1)^2 + 100*x0(3)^2;
figure;
subplot(1,2,1);plot3(x0(1),x0(2),x0(3),'go');hold on;grid on;
subplot(1,2,2);plot(iter,f0,'ro');hold on;grid on;xlabel('# iter');ylabel('f');

% Solve the NLP
sol    = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);

%% Solve the NLP and print solution
f_opt     = full(sol.f)           % >> 0
x_opt     = full(sol.x)           % >> [0; 1; 0]
lam_x_opt = full(sol.lam_x)       % >> [0; 0; 0]
lam_g_opt = full(sol.lam_g)       % >> 0
