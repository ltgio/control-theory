% EXAMPLE BUG FREE
% In this example, we fit a nonlinear model to measurements
% Model: Chemical Reactor
%      xdot1 = u1 - k1*x1
%      xdot2 = u1*u2/x1 - u1*x2/x1 - k2*x2
% x = [x1;x2] | u = [u1;u2] | p = [ka;kb]    
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

% Version:     Matlab 2014b/optistack-casadi 3.0.0
% Author:      Marie Curie PhD student Giovanni Licitra
% Data:        09-09-2016

clear all;close all;clc;
import casadi.*
%% SETTINGS ===============================================================
N  = 500;                           % Number of samples
fs = 50;                            % Sampling frequency [hz]
t  = linspace(0,(N-1)*(1/fs),N)';   % time array

N_steps_per_sample = 4;   
dt = 1/fs/N_steps_per_sample;       % integration step for ode

nx = 2;                             % n states
nu = 2;                             % n inputs
np = 2;                             % n parameters

rng(1)     
%% Model parameters =======================================================
theta_truth = [0.1;0.5];   % True parameters
theta_guess = [0.3;0.4];   % guess for NLP
scale       = [1e-1;1e-1]; % scaling factor

%% Model generation via casADi/OPTistack ==================================
x1 = MX.sym('x1');
x2 = MX.sym('x2');
u1 = MX.sym('u1');
u2 = MX.sym('u2');

states   = [x1;x2];
controls = [u1;u2];

% define uknow parametes as design variable
ka    = MX.sym('ka'); 
kb    = MX.sym('kb');
theta = [ka;kb];            % store uknow parameters 

% xdot = f(x,u,p) <==> rhs = f(x,u,p)
rhs = [u1-ka*x1;               % xdot = f(x,u,p)
       u1*u2/x1-u1*x2/x1-kb*x2];
% Form an ode function
ode = Function('ode',{states,controls,theta},{rhs});
ode.print_dimensions;
%% numerical evaluation ===================================================
x_test     = [0.1,-0.1];
u_test     = [0.2,-0.1];
theta_test = [0.1,0.5];
f_out      = full(ode(x_test,u_test,theta_test));   

%% build integrator: RK4 ==================================================
k1 = ode(states          ,controls,theta);
k2 = ode(states+dt/2.0*k1,controls,theta);
k3 = ode(states+dt/2.0*k2,controls,theta);
k4 = ode(states+dt*k3    ,controls,theta);
xf = states + dt/6.0*(k1+2*k2+2*k3+k4);
% Create a function that simulates one step propagation in a sample
one_step = Function('one_step',{states, controls, theta},{xf});

X = states;
for i=1:N_steps_per_sample
    X = one_step(X, controls, theta);
end

% Create a function that simulates all step propagation on a sample
one_sample = Function('one_sample',{states, controls, theta}, {X});
% speedup trick: expand into scalar operations
one_sample = one_sample.expand();

%% Compute Forward Simulation =============================================
% choose number of simulation step
all_samples = one_sample.mapaccum('all_samples', N);               % for casadi up to v3.0.0
%all_samples = one_sample.mapaccum('map', N/10).mapaccum('map',10); % for casadi v3.0.1

%% Choose an excitation signal %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% experiment 1
u_data1  = 2*chirp(t,1,10,5,'logarithmic'); % Choose signal excitation [chirp]
u_data2  = 2*randn(N,1);                    % Choose signal excitation [random noise]
Udata    = [u_data1 u_data2]';              % 
%x0       = DM([1,-1]);                     % Initial Condition x0 = [0;0]; [nx = 2]
x0       = [1,-1];                          % Initial Condition x0 = [0;0]; [nx = 2]

% perform forward simulation
X_truth = all_samples(x0, Udata, repmat(theta_truth,1,N));
X_truth = full(X_truth);
X_truth = [x0',X_truth]; % add the initial condition
X_truth = X_truth(:,1:end-1);
% OPTION 1: gaussian noise
ny = 0.02*randn(N,nx);                       
% sum noise measurements to the state
y_data = DM(X_truth' + ny); 

%% Set Identification Algorithm =========================================== 
disp('Multiple shooting...')
X  = MX.sym('X',nx, N); % continuity for exp1
one_sample_map = one_sample.map(N, 'openmp');
Xn = one_sample_map(X, Udata, repmat(theta.*scale,1,N));

% gap-closing constraints
gaps = Xn(:,1:end-1) - X(:,2:end);

% error = y_data - x_symbolic  ============================================ 
e = vec(y_data'-X);

%% multiple shooting casadi ===============================================
w      = {theta;vec(X)};
w0     = [theta_guess; vec(y_data')];
lbw    = -inf*ones(size(w0));
ubw    =  inf*ones(size(w0));
J      = 0.5*dot(e,e);
g      = {vec(gaps)};
lbg    = zeros(nx*(N-1),1);
ubg    = zeros(nx*(N-1),1);

w  = vertcat(w{:});
g  = vertcat(g{:});

%% use Gauss-Newton Hessian
Jw      = jacobian(e,w);
H       = triu(Jw'*Jw);
sigma   = MX.sym('sigma');
hessLag = Function('nlp_hess_l',struct('x',w,'lam_f',sigma, 'hess_gamma_x_x',sigma*H),...
                     char('x','p','lam_f','lam_g'), char('hess_gamma_x_x'));

%% Create an NLP solver
prob   = struct('f', J, 'x', w, 'g', g);
% option IPOPT
opts                             = struct;
opts.ipopt.linear_solver         = 'ma27';
opts.ipopt.max_iter              = 50;
opts.ipopt.hessian_approximation = 'exact';  % 3 iteration
%opts.hess_lag                    = hessLag; % 5 iteration

% Solve the NLP
solver  = nlpsol('solver', 'ipopt', prob,opts);
w_sol   = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);

theta_est = scale.*full(w_sol.x(1:np)); % retrieve parameters
X_sol  = full(w_sol.x(np+1:end));       % retrieve states

X_est = [];
for i=1:nx
  X_est = [X_est, X_sol(i:nx:end)];
end

%X_est = full(all_samples(x0, Udata, repmat(theta_est,1,N)));
%% Plot residuals [only time information] =================================
figure;
title('data fitting');
plot(t,full(y_data)  ,'LineWidth',2,'Color','b');hold on;grid on;
plot(t,full(X_truth)','LineWidth',2.5,'Color','r');
plot(t,X_est         ,'LineWidth',2,'Color','g');
legend('y_{1}','y_{2}','x true_{1}','x true_{2}','x est_{1}','x est_{2}');
xlabel('time [s]');ylabel('y_measurement vs y_simulated');

% Print some information ==================================================
disp('');
disp('True parameters')
str = '%s* = %f \n';Cdisp = {'ka','kb';theta_truth(1),theta_truth(2)};
disp(sprintf(str,Cdisp{:}));

disp('Estimated parameters')
str = '%s* = %f \n';Cdisp = {'ka','kb';theta_est(1),theta_est(2)};
disp(sprintf(str,Cdisp{:}));

% Inspect Jacobian sparsity
Jacobian = jacobian(g, w);

% Inspect Hessian of the Lagrangian sparsity
Lambda     = MX.sym('lam', g.sparsity());
Lagrancian = w_sol.f + dot(Lambda, g);
Hessian    = hessian(Lagrancian, w);

figure;
subplot(1,2,1);title('Jacobian sparsity');hold on;
spy(sparse(DM.ones(Jacobian.sparsity())))
subplot(1,2,2);title('Hessian sparsity');hold on;
spy(sparse(DM.ones(Hessian.sparsity())))

