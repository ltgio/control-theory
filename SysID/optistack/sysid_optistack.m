clc;clear all;close all;
import casadi.*

% In this example, we fit a nonlinear model to measurements
%
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
N = 10000;  % Number of samples
fs = 610.1; % Sampling frequency [hz]

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

states = [y;dy];
controls = u;

M = optivar();
c = optivar();
k = optivar();
k_NL = optivar();

params = [M;c;k;k_NL];

rhs = [dy; 
       (u-k_NL*y.^3-k*y-c*dy)/M];

% Form an ode function
ode = Function('ode',{states,controls,params},{rhs});

%%%%%%%%%%%% Creating a simulator %%%%%%%%%%
N_steps_per_sample = 10;
dt = 1/fs/N_steps_per_sample;

% Build an integrator for this system: Runge Kutta 4 integrator
k1 = ode(states          ,controls,params);
k2 = ode(states+dt/2.0*k1,controls,params);
k3 = ode(states+dt/2.0*k2,controls,params);
k4 = ode(states+dt*k3    ,controls,params);

states_final = states+dt/6.0*(k1+2*k2+2*k3+k4);

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
u_data = 0.1*rand(N,1);

x0 = DM([0,0]);
X_measured = all_samples(x0, u_data, repmat(param_truth,1,N));
sigma_y    = 0.01;
y_data     = X_measured(1,:)' + sigma_y*randn(N,1);
%%%%%%%%%%%% Identifying the simulated system: single shooting strategy %%%%%%%%%%
% Note, it is in general a good idea to scale your decision variables such
% that they are in the order of ~0.1..100
X_symbolic = all_samples(x0, u_data, repmat(params.*scale,1,N));

e = y_data - X_symbolic(1,:)';

M.setInit(param_guess(1));
c.setInit(param_guess(2));
k.setInit(param_guess(3));
k_NL.setInit(param_guess(4));

options = struct;
options.codegen = false;

disp('Single shooting...')

% Hand in a vector objective -> interpreted as 2-norm
% such t hat Gauss-Newton can be performed
optisolve(e,{},options);
M_est    = optival(M)*1e-6;
c_est    = optival(c)*1e-4;
k_est    = optival(k);
k_NL_est = optival(k_NL);

param_est_SS = [M_est,c_est,k_est,k_NL_est];
%rms(optival(e))
rms_SS = sqrt(mean(optival(e).^2)); % rms is part of the signal toolbox

%%%%%%%%%%%% Identifying the simulated system: multiple shooting strategy %%%%%%%%%%
disp('Multiple shooting...')

X = optivar(2, N);

params_scale = [1e-6*M;c*1e-4;k;k_NL];

Xn = one_sample.map({X, u_data', params_scale});
Xn = Xn{1};

% gap-closing constraints
gaps = Xn(:,1:end-1)-X(:,2:end);
g = gaps == 0;

e = (y_data-X(1,:)');

M.setInit(param_guess(1));
c.setInit(param_guess(2));
k.setInit(param_guess(3));
k_NL.setInit(param_guess(4));
X.setInit([ y_data [diff(full(y_data))*fs;0]]');

options         = struct;
options.codegen = false;

% Hand in a vector objective -> interpreted as 2-norm 
% such that Gauss-Newton can be performed
optisolve(e,{g},options);

M_est    = optival(M)*1e-6;
c_est    = optival(c)*1e-4;
k_est    = optival(k);
k_NL_est = optival(k_NL);

param_est_MS = [M_est,c_est,k_est,k_NL_est];

% rms(optival(e))
rms_MS = sqrt(mean(optival(e).^2)); % rms is part of the signal toolbox

disp('theta truth')
disp(param_truth')

disp('estimates: single shooting')
disp(param_est_SS)
disp(rms_SS)
disp('estimates: multiple shooting')
disp(param_est_MS)
disp(rms_MS)