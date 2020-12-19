% Title:       Input Output Non Linear Least Square for DC motor
% Version:     Matlab 2014b
% Author:      Marie Curie PhD student Giovanni Licitra
% Data:        09-09-2016

clear all;close all;clc;
import casadi.*
%% SETTINGS ===============================================================
N  = 10;                           % Number of samples
fs = 50;                            % Sampling frequency [hz]
t  = linspace(0,(N-1)*(1/fs),N)';   % time array

N_steps_per_sample = 4;   
dt = 1/fs/N_steps_per_sample;       % integration step for ode

nx = 2;                             % n states
nu = 1;                             % n inputs
np = 1;                             % n parameters

rng(1)     
%% Model parameters =======================================================
theta_truth = [1];   % True parameters
theta_guess = [0.8]; % guess for NLP
scale       = [1];   % scaling factor

%% Model generation via casADi/OPTistack ==================================
x1 = MX.sym('x1');
x2 = MX.sym('x2');
u  = MX.sym('u');

states   = [x1;x2];
controls = [u];

% define uknow parametes as design variable
tau   = MX.sym('tau'); 
theta = [tau];            % store uknow parameters 

% xdot = f(x,u,p) <==> rhs = f(x,u,p)
G = 0.25;                         % gain DC motor
rhs = [x2                         % xdot = f(x,u,p)
       x1 - (1/tau)*x2 + (G/tau)*u];
% Form an ode function
ode = Function('ode',{states,controls,theta},{rhs});
ode.print_dimensions;
%% numerical evaluation ===================================================
x_test     = [0.1,-0.1];
u_test     = [0.2];
theta_test = [0.1];
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
u_data0  = 40*randn(N,1);    % Choose signal excitation [chirp]
sigma_u  = 0.2;
u_data   = u_data0 + sigma_u*randn(N,nu);
u_data   = [u_data]';        % 
x0       = DM([1,-1]);       % Initial Condition x0 = [0;0]; [nx = 2]

% perform forward simulation
X_truth = all_samples(x0, u_data, repmat(theta_truth,1,N));
X_truth = [x0',X_truth];   % add the initial condition
X_truth = X_truth(:,1:end-1);

% gaussian noise
sigma_y = 0.25;
ny = sigma_y*randn(nx,N);                       
% sum noise measurements to the state
y_data = X_truth + ny; 

%% Set Identification Algorithm =========================================== 
disp('Multiple shooting...')
X  = MX.sym('X',nx, N); % continuity for exp1
U  = MX.sym('U',nu, N); % continuity for exp1

Xn = one_sample.map({X, U, theta.*scale});
Xn = Xn{1};

% gap-closing constraints
gaps = Xn(:,1:end-1) - X(:,2:end);

% error = y_data - x_symbolic  ============================================ 
% NB: you assume same noise measurements
e_y = vec( (y_data-X)./sigma_y );
e_u = vec( (u_data-U)./sigma_u );
e   = [e_y;e_u];

%% multiple shooting casadi ===============================================
w      = {theta;vec(X);vec(U)};
w0     = [theta_guess; vec(y_data); vec(DM(u_data))];
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
opts.ipopt.max_iter              = 500;
opts.ipopt.hessian_approximation = 'exact';  % 3 iteration
%opts.hess_lag                    = hessLag; % 5 iteration

% Solve the NLP
solver  = nlpsol('solver', 'ipopt', prob,opts);
w_sol   = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);

theta_est = scale.*full(w_sol.x(1:np)); % retrieve parameters
X_est  = full(w_sol.x(np+1:np+2+nx*(N-1)));    % retrieve states
U_est  = full(w_sol.x(np+3+nx*(N-1): end));
X_est1 = X_est(1:nx:end);  % state 1
X_est2 = X_est(2:nx:end);  % state 2
X_est  = [X_est1,X_est2];

%X_est = full(all_samples(x0, Udata, repmat(theta_est,1,N)));
%% Plot residuals [only time information] =================================
figure;
subplot(2,1,1);
title('data fitting');
plot(t,full(y_data)  ,'bo');hold on;grid on;
plot(t,full(X_truth)','LineWidth',2.5,'Color','r');
plot(t,X_est         ,'LineWidth',2,'Color','g');
legend('y_{1}','y_{2}','x true_{1}','x true_{2}','x est_{1}','x est_{2}');
xlabel('time [s]');ylabel('y_measurement vs y_simulated');
subplot(2,1,2);
plot(t,u_data0  ,'ro');hold on;grid on;
stairs(t,u_data   ,'LineWidth',2,'Color','b');hold on;grid on;
stairs(t,U_est   ,'LineWidth',2,'Color','g');hold on;grid on;
legend('u true','u noisy','u est');

% Print some information ==================================================
disp('');
disp('True parameters')
str = '%s* = %f \n';Cdisp = {'tau';theta_truth(1)};
disp(sprintf(str,Cdisp{:}));

disp('Estimated parameters')
str = '%s* = %f \n';Cdisp = {'tau';theta_est(1)};
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

