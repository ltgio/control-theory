%% Bug free
%  Parameter estimation for Lotka-Volterra
clc;clear all;close all
import casadi.*
rng(1)

nx = 2;
nu = 0;

T = linspace(0, 10, 11)';
N = length(T);
p_guess = [1;1];

x0 = [1;1];

yN = DM([1.0, 0.9978287, 2.366363, 6.448709, 5.225859, 2.617129,1.324945, 1.071534, 1.058930, 3.189685, 6.790586;
      1.0, 2.249977, 3.215969, 1.787353, 1.050747, 0.2150848,0.109813, 1.276422, 2.493237, 3.079619, 1.665567]');

% define the standard deviation of the noise
sigma_x1 = 0.1;
sigma_x2 = 0.2;

%% define grey-box model
x     = MX.sym('x', 2);
alpha = 1.0;
gamma = 1.0;

p   = MX.sym('p', 2); 
f   = [-alpha * x(1) + p(1) * x(1) * x(2); 
        gamma * x(2) - p(2) * x(1) * x(2)];
phi = x;
%system = cp.system.System(x = x, p = p, f = f, phi = phi)
% Form an ode function
ode = Function('ode',{x,p},{f});

% # The weightings for the measurements errors given to casiopeia are calculated
% # from the standard deviations of the measurements, so that the least squares
% # estimator ist the maximum likelihood estimator for the estimation problem.

wv      = zeros(length(yN),2);
wv(:,1) = (1.0 / sigma_x1^2);
wv(:,2) = (1.0 / sigma_x2^2);

% pe = cp.pe.LSq(system = system, time_points = T, xinit = yN, \
%     ydata = yN, wv = wv, discretization_method = "collocation")

Ts = 1;
fs = 1/Ts;
N_steps_per_sample = 4;
dt = 1/fs/N_steps_per_sample;

% Build an integrator for this system: Runge Kutta 4 integrator
k1 = ode(x,          p);
k2 = ode(x+dt/2.0*k1,p);
k3 = ode(x+dt/2.0*k2,p);
k4 = ode(x+dt*k3,    p);

states_final = x+dt/6.0*(k1+2*k2+2*k3+k4);

% Create a function that simulates one step propagation in a sample
one_step = Function('one_step',{x, p},{states_final});

X = x;
for i=1:N_steps_per_sample
    X = one_step(X, p);
end

% Create a function that simulates all step propagation on a sample
one_sample = Function('one_sample',{x, p}, {X});
% speedup trick: expand into scalar operations
one_sample = one_sample.expand();

%%%%%%%%%%%% Identifying the simulated system: multiple shooting strategy %%%%%%%%%%
disp('Multiple shooting...')
X  = MX.sym('X',nx, length(yN));
Xn = one_sample.map({X, p});
Xn = Xn{1};

% gap-closing constraints
gaps = Xn(:,1:end-1)-X(:,2:end); % gaps == 0;

% define fitting
e = vec(yN-X');

% Alternative way (less efficient)
% e  = MX.zeros(2*length(yN),1);
% e1 = yN(:,1) - X(1,:)';  
% e2 = yN(:,2) - X(2,:)';
% e(1:nx:end-1) = e1;
% e(2:nx:end)   = e2;

% e(1:nx:end-1) = (1/sigma_x1^2)*e1;
% e(2:nx:end)   = (1/sigma_x2^2)*e2;

%% multiple shooting casadi only =========================================
% problem
w      = {p      ; vec(X) };
w0     = [p_guess; vec(yN')];
lbw    = -inf*ones(size(w0));
ubw    =  inf*ones(size(w0));
J      =  0.5*dot(e,e); %0.5*(e'*e);
g      = {vec(gaps)};
lbg    = zeros(nx*(N-1),1);
ubg    = zeros(nx*(N-1),1);

w = vertcat(w{:});
g = vertcat(g{:});

% implement the Gauss-Newton Hessian
Jw = jacobian(e,w);
H  = triu(Jw'*Jw);

sigma   = MX.sym('sigma');
hessLag = Function('nlp_hess_l',struct('x',w,'lam_f',sigma, 'hess_gamma_x_x',sigma*H),...
                     char('x','p','lam_f','lam_g'), char('hess_gamma_x_x'));

% Create an NLP solver
prob   = struct('f', J, 'x', w, 'g', g);
% option IPOPT
opts                     = struct;
opts.ipopt.linear_solver = 'ma86';
opts.ipopt.max_iter      = 500;
%opts.hess_lag            = hessLag;
opts.ipopt.hessian_approximation = 'exact';
%opts.monitor = char('nlp_g','nlp_hess_l','nlp_grad_f','nlp_jac_g');

% Solve the NLP
solver = nlpsol('solver', 'ipopt', prob,opts);
sol    = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);
w_sol  = full(sol.x);

%param_truth'
p_est = w_sol(1:length(p_guess))

X1 = w_sol(3:nx:end-1);
X2 = w_sol(4:nx:end);

X_est = [X1,X2];

plot(X_est);hold on;grid on;
plot(full(yN),'o');ylabel('y');xlabel('t[k]');

% Inspect Jacobian sparsity
Jacobian = jacobian(g, w);
figure;spy(sparse(DM.ones(Jacobian.sparsity())))
title('Inspect Jacobian sparsity')

% Inspect Hessian of the Lagrangian sparsity
Lambda     = MX.sym('lam', g.sparsity());
Lagrancian = sol.f + dot(Lambda, g);
Hessian    = hessian(Lagrancian, w);
figure;spy(sparse(DM.ones(Hessian.sparsity())));
title('Inspect Hessian of the Lagrangian sparsity');
