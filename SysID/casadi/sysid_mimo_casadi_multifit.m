%% EXAMPLE BUG FREE
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
N  = 50;                          % Number of samples
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
ode.printDimensions();
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
all_samples = one_sample.mapaccum('all_samples', N);

%% Choose an excitation signal %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% experiment 1
u_data1     = 1*chirp(t,1,10,5,'logarithmic');     % Choose signal excitation [chirp]
u_data2     = 0.05*randn(N,1);                     % Choose signal excitation [random noise]
Udata1      = [u_data1 u_data2]';                  % 
x0_1        = [1,-1];                          % Initial Condition x0 = [0;0]; [nx = 2]

% perform forward simulation
X_truth1  = all_samples(x0_1, Udata1, repmat(theta_truth,1,N));
X_truth1 = full(X_truth1);
X_truth1 = [x0_1',X_truth1]; % add the initial condition
X_truth1 = X_truth1(:,1:end-1);
% OPTION 1: gaussian noise
ny = 0.03*randn(N,nx);                       
% sum noise measurements to the state
y_data1 = X_truth1' + ny; 

%% Choose an excitation signal %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% experiment 2
u_data1     = 0.5*chirp(t,1,10,5,'logarithmic');          % Choose signal excitation [chirp]
u_data2     = 0.2*rand(N,1);                     % Choose signal excitation [random noise]
Udata2      = [u_data1 u_data2]';                  % 
x0_2        = [-0.5,0.5];                          % Initial Condition x0 = [0;0]; [nx = 2]

% perform forward simulation
X_truth2 = all_samples(x0_2, Udata2, repmat(theta_truth,1,N));
X_truth2 = full(X_truth2);
X_truth2 = [x0_2;X_truth2']'; % add the initial condition
X_truth2 = X_truth2(:,1:end-1);
% OPTION 1: gaussian noise
ny = 0.03*randn(N,nx);                       
% sum noise measurements to the state
y_data2 = X_truth2' + ny;

%% Set Identification Algorithm =========================================== 
disp('Multiple shooting casadi only')

X1 = MX.sym('X1',nx, N); % continuity for exp1
X2 = MX.sym('X2',nx, N); % continuity for exp2

theta_scale = theta.*scale; % params contains my design variable 

Xn1 = one_sample.map({X1, Udata1, theta_scale});Xn1 = Xn1{1};
Xn2 = one_sample.map({X2, Udata2, theta_scale});Xn2 = Xn2{1};

% gap-closing constraints
gaps1 = Xn1(:,1:end-1) - X1(:,2:end);
gaps2 = Xn2(:,1:end-1) - X2(:,2:end);

% error = y_data - x_symbolic  ============================================ 
e_exp1 = vec(y_data1'-X1);
e_exp2 = vec(y_data2'-X2);

%% multiple shooting casadi only ============================ 2 experiments
Nexp   = 2;                      % # of experiments
w      = {theta;vec(X1);vec(X2)};
w0     = [theta_guess; vec(DM(y_data1')) ; vec(DM(y_data2'))];
lbw    = -inf*ones(size(w0));
ubw    =  inf*ones(size(w0));
J      = 0.5*(e_exp1'*e_exp1)+0.5*(e_exp2'*e_exp2);
g      = {vec(gaps1);vec(gaps2)};
lbg    = zeros(Nexp*nx*(N-1),1);
ubg    = zeros(Nexp*nx*(N-1),1);

w  = vertcat(w{:});
g  = vertcat(g{:});

%% use Gauss-Newton Hessian
Jw      = jacobian([e_exp1;e_exp2],w);
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
X_est     = full(w_sol.x(np+1:end));    % retrieve states

X_est1    = X_est(1:Nexp*N);
X_est_1_1 = X_est1(1:nx:end);
X_est_1_2 = X_est1(2:nx:end);
X_est1    = [X_est_1_1,X_est_1_2];

X_est2    = X_est(Nexp*N+1:end);
X_est_2_1 = X_est2(1:nx:end);
X_est_2_2 = X_est2(2:nx:end);
X_est2    = [X_est_2_1,X_est_2_2];

%% Plot residuals [only time information] =================================
figure;
subplot(1,2,1);
title('Experiment 1');
plot(t,full(y_data1)  ,'LineWidth',2,'Color','b');hold on;grid on;
plot(t,full(X_truth1)','LineWidth',2.5,'Color','r');
plot(t,X_est1         ,'LineWidth',2,'Color','g');
legend('y_{1}','y_{2}','x true_{1}','x true_{2}','x est_{1}','x est_{2}');
xlabel('time [s]');ylabel('y_measurement vs y_simulated');
subplot(1,2,2);
title('Experiment 2');
plot(t,full(y_data2)  ,'LineWidth',2,'Color','b');hold on;grid on;
plot(t,full(X_truth2)','LineWidth',2.5,'Color','r');
plot(t,X_est2         ,'LineWidth',2,'Color','g');
xlabel('time [s]');ylabel('y_measurement vs y_simulated');
legend('y_{1}','y_{2}','x true_{1}','x true_{2}','x est_{1}','x est_{2}');

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

