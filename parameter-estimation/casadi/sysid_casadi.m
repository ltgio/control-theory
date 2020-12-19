%% Bug Present
clc;clear all;close all;
import casadi.*

% In this example, we fit a nonlinear model to measurements
% This example uses more advanced constructs than the vdp* examples:
% Since the number of control intervals is potentially very large here,
% we use memory-efficient Map and MapAccum, in combination with
% codegeneration.
%
% We will be working with a 2-norm objective:
% || y_measured - y_simulated ||_2^2
%
% This form is well-suited for the Gauss-Newton Hessian approximation.

%%%%%%%%%%% SETTINGS %%%%%%%%%%%%%%%%%%%%%
N  = 1000;         % Number of samples
fs = 610.1;        % Sampling frequency [hz]

param_truth = [5.625e-6;2.3e-4;1;4.69];
param_guess = [5;2;1;5];
scale       = [1e-6;1e-4;1;1];
rng(1)
%%%%%%%%%%%% MODELING %%%%%%%%%%%%%%%%%%%%%
nx = 2;
nu = 1;

y  = MX.sym('y');
dy = MX.sym('dy');
u  = MX.sym('u');

states   = [y;dy];
controls = u;

M    = MX.sym('M',1);
c    = MX.sym('c',1);
k    = MX.sym('k',1);
k_NL = MX.sym('K_NL',1);

params = [M;c;k;k_NL];

rhs    = [dy; 
          (u-k_NL*y.^3-k*y-c*dy)/M];

% Form an ode function
ode = Function('ode',{states,controls,params},{rhs});

%%%%%%%%%%%% Creating a simulator %%%%%%%%%%
N_steps_per_sample = 10;
dt = 1/fs/N_steps_per_sample;

% Build an integrator for this system: Runge Kutta 4 integrator
k1 = ode(states,          controls,params);
k2 = ode(states+dt/2.0*k1,controls,params);
k3 = ode(states+dt/2.0*k2,controls,params);
k4 = ode(states+dt*k3,    controls,params);

states_final = states + dt/6.0*(k1+2*k2+2*k3+k4);

% Create a function that simulates one step propagation in a sample
one_step = Function('one_step',{states, controls, params},{states_final});

X = states;
for i=1:N_steps_per_sample
    X = one_step(X, controls, params);
end

% Create a function that simulates all step propagation on a sample
one_sample = Function('one_sample',{states, controls, params}, {X});

% speedup trick: expand into scalar operations
one_sample = one_sample.expand();

%%%%%%%%%%%% Simulating the system %%%%%%%%%%
all_samples = one_sample.mapaccum('all_samples', N);

% Choose an excitation signal
u_data     = 0.1*rand(N,1);
x0         = DM([0,0]);
X_measured = all_samples(x0, u_data, repmat(param_truth,1,N));
sigma_y    = 0.01;
y_data     = X_measured(1,:)' + sigma_y*randn(N,1);

%%%%%%%%%%%% Identifying the simulated system: single shooting strategy %%%%%%%%%%

% Note, it is in general a good idea to scale your decision variables such
% that they are in the order of ~0.1..100
y0         = y_data(1);
X_symbolic = all_samples(y_data(1), u_data, repmat(params.*scale,1,N));
e          = y_data-X_symbolic(1,:)';

%% single shooting using casadi only
disp('single shooting...')
w      = {params};
w0     = param_guess;
lbw    = -inf*ones(size(param_guess));
ubw    =  inf*ones(size(param_guess));
J      = 0.5*dot(e,e);
g      = {};
lbg    = [];
ubg    = [];

% Create an NLP solver
prob   = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
% option IPOPT
opts                     = struct;
opts.ipopt.linear_solver = 'ma27';
opts.ipopt.max_iter      = 500;

% Solve the NLP
solver = nlpsol('solver', 'ipopt', prob,opts);
sol    = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);
w_sol  = full(sol.x);
w_sol(1) = w_sol(1)*1e-6;
w_sol(2) = w_sol(2)*1e-4;

param_est_SS = w_sol';

%%%%%%%%%%%% Identifying the simulated system: multiple shooting strategy %%%%%%%%%%
disp('Multiple shooting casadi')
X  = MX.sym('X', nx, N);
Xn = one_sample.map({X, u_data', params.*scale});
Xn = Xn{1};

% gap-closing constraints
gaps = Xn(:,1:end-1)-X(:,2:end);

% error = y_data - x_symbolic  ============================================
e    = (y_data'-X(1,:));

%% multiple shooting casadi ===============================================
w      = {params;vec(X)};
w0     = [param_guess; vec([y_data,[diff(full(y_data))*fs;0]]) ];
%y_data = [y_data, [diff(full(y_data))*fs;0]];
%w0     = [param_guess; vec(y_data) ];
lbw    = -inf*ones(size(w0));
ubw    =  inf*ones(size(w0));
J      = 0.5*dot(e,e);
g      = {vec(gaps)};
lbg    = zeros(nx*(N-1),1);
ubg    = zeros(nx*(N-1),1);

w = vertcat(w{:});
g = vertcat(g{:});

Jw = jacobian(e,w);
H  = triu(Jw'*Jw);

sigma   = MX.sym('sigma');
hessLag = Function('nlp_hess_l',struct('x',w,'lam_f',sigma, 'hess_gamma_x_x',sigma*H),...
                   char('x','p','lam_f','lam_g'), char('hess_gamma_x_x'));

% Create an NLP solver
prob   = struct('f', J, 'x', w, 'g', g);
% option IPOPT
opts                     = struct;
opts.ipopt.linear_solver = 'ma27';
opts.ipopt.max_iter      = 50;
opts.ipopt.hessian_approximation = 'exact'; 
%opts.hess_lag            = hessLag;

% Solve the NLP
solver = nlpsol('solver', 'ipopt', prob,opts);
sol    = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);
w_sol  = full(sol.x);
w_sol(1) = w_sol(1)*1e-6;
w_sol(2) = w_sol(2)*1e-4;
param_est_MS = w_sol(1:4)';

disp('theta truth')
disp(param_truth')
disp('estimates: single shooting')
disp(param_est_SS)
disp('estimates: multiple shooting')
disp(param_est_MS)