% Title:   Multiple Shooting Rocket Time Optimal
% Version: Matlab 2014b/casadi3.0.0
% Author:  Marie Curie PhD student Giovanni Licitra
% Data:    17-09-2016

clear all;clc;close all;
import casadi.*

%% States =================================================================

px   = SX.sym('p' ,1); % position 
vx   = SX.sym('v' ,1); % velocity
mF   = SX.sym('mF',1); % mass fuel
x    = [px;vx;mF];

u    = SX.sym('u',1);  % thrust

nx   = length(x);
nu   = length(u);

% define ode
xdot = [vx;
        u/(30+mF);
        -u^2];
       
% Objective term
Q   = eye(nx);
R   = eye(nu);
L   = x'*Q*x + + u'*R*u;
% Continuous time dynamics
f  = Function('f', {x, u}, {xdot, L} ,...
     char('states', 'controls'), char('ode', 'Mayer Term'));

%% ========================================================================
% Control discretization
% Time horizon
T       = MX.sym('T',1); % Time is an design variables
N       = 40;            % number of control intervals
M       = 4;             % RK4 steps per interval
DT      = T/N/M;
X0      = MX.sym('X0', nx);
U       = MX.sym('U' , nu);
X       = X0;
Q       = 0;

for j=1:M
    [k1, k1_q] = f(X            , U);
    [k2, k2_q] = f(X + DT/2 * k1, U);
    [k3, k3_q] = f(X + DT/2 * k2, U);
    [k4, k4_q] = f(X + DT   * k3, U);
    X = X + DT/6*(k1   + 2*k2   + 2*k3   + k4  );
    Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
end
F  = Function('F', {X0, U, T}, {X, Q});

%% Start with an empty NLP ================================================
w      = {};      % solution array
w0     = [];      % initial guess array
lbw    = [];      % lower bound for w
ubw    = [];      % upper bound for w
J      = 0; 
g      = {};
lbg    = [];
ubg    = [];

% "Lift" initial conditions
x0     = [50;0;10];

X0     = MX.sym('X0', nx);
w      = {w{:}, X0};

lbw    = [lbw;  x0];
ubw    = [ubw;  x0];
w0     = [w0;   x0];

u_min  = [-inf];
u_max  = [ inf];

x_min = [-inf; -inf; -inf];
x_max = [ inf;  inf;  inf];

% Formulate the NLP
Xk = X0;
for k=0:N-1
    % New NLP variable for the control
    Uk  = MX.sym(['U_' num2str(k)],nu);
    w   = {w{:}, Uk};
    % bound on input   
    lbw = [lbw;  u_min];
    ubw = [ubw;  u_max];
    w0  = [w0 ;  zeros(nu,1)];

    % Integrate till the end of the interval
    [Xk_end, Jk] = F(Xk, Uk, T);
    J = J + Jk;

    % New NLP variable for state at end of interval
    Xk  = MX.sym(['X_' num2str(k+1)], nx);
    w   = {w{:}, Xk};
    lbw = [lbw; x_min];
    ubw = [ubw; x_max];
    w0  = [w0 ; zeros(nx,1)];
        
    % Add equality constraint
    g   = {g{:}, Xk_end - Xk};
    lbg = [lbg; zeros(nx,1)];
    ubg = [ubg; zeros(nx,1)];
end

% final condition
lbw(end-nx+1:end) = [0;0;  4];
ubw(end-nx+1:end) = [0;0;inf];

% Time constraint
w0  = [w0 ; 100];
lbw = [lbw; 0];
ubw = [ubw; inf];

% Create an NLP solver
prob   = struct('f', T, 'x', vertcat(w{:},T), 'g', vertcat(g{:}));
% option IPOPT
% see: http://www.coin-or.org/Ipopt/documentation/node39.html
opts                             = struct;
opts.ipopt.max_iter              = 500;
opts.ipopt.linear_solver         = 'mumps';
opts.ipopt.hessian_approximation = 'exact';

%% Solve the NLP
solver = nlpsol('solver', 'ipopt', prob,opts);
sol    = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);
w_opt  = full(sol.x);

% Plot the solution
p_opt    = w_opt(1:nx+nu:end);
v_opt    = w_opt(2:nx+nu:end);
mF_opt   = w_opt(3:nx+nu:end);
u_opt    = w_opt(4:nx+nu:end-1);
T        = w_opt(end);

figure;
time = linspace(0, T, N+1);
subplot(2,2,1);plot(time,p_opt        ,'Color','r','LineWidth',2);legend('p[t]');grid on;
subplot(2,2,2);plot(time,v_opt        ,'Color','b','LineWidth',2);legend('v[t]');grid on;
subplot(2,2,3);plot(time,mF_opt       ,'Color','g','LineWidth',2);legend('mF[t]');grid on;
subplot(2,2,4);stairs(time,[u_opt;nan],'Color','m','LineWidth',2);legend('u[t]');grid on;

% Inspect Jacobian sparsity
sol.g        = vertcat(g{:});
sol.w        = vertcat(w{:});
sol.Jacobian = jacobian(sol.g, sol.w);

% Inspect Hessian of the Lagrangian sparsity
sol.Lambda     = MX.sym('lam', sol.g.sparsity());
sol.Lagrancian = sol.f + dot(sol.Lambda, sol.g);
sol.Hessian    = hessian(sol.Lagrancian, sol.w);

figure;
subplot(1,2,1);spy(sparse(DM.ones(sol.Jacobian.sparsity())))
               title('Jacobian sparsity');              
subplot(1,2,2);spy(sparse(DM.ones(sol.Hessian.sparsity())))
               title('Hessian sparsity');