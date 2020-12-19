% Linear MPC: state tracking and output tracking 
% Version: Matlab 2016b/casadi3.0.0
% Author:  Marie Curie PhD student Giovanni Licitra

clear all;clc;close all;
import casadi.*

%% Parameters =============================================================
mass  = 1;           % massa veivolo [kg]

%% States =================================================================
y   = SX.sym('y' ,1); 
vy  = SX.sym('vy',1);
z   = SX.sym('z' ,1);
vz  = SX.sym('vz',1);

x  = [y;vy;z;vz];

Fy = SX.sym('Fy',1);
Fz = SX.sym('Fz',1);
u  = [Fy;Fz];

% reference for trajectory preview
r_y   = SX.sym('r_y' ,1); 
r_vy  = SX.sym('r_vy',1);
r_z   = SX.sym('r_z' ,1);
r_vz  = SX.sym('r_vz',1);

r  = [r_y;r_vy;r_z;r_vz];

nx  = length(x);
nu  = length(u);

% define ode
A = [0, 1 , 0 , 0;
     0, 0 , 0 , 0;
     0, 0 , 0 , 1;
     0, 0 , 0 , 0];

B = [0 , 0;
     1 , 0;
     0 , 0;
     0 , 1];

xdot = A*x + B*u;
 
% Objective term
%      | y|vy| z|vz|
xref = [ 1; 0;-1; 0];

Q = 10*eye(nx);
R = 0.1;eye(nu);
L = (x-r)'*Q*(x-r) + u'*R*u;

% Continuous time dynamics
f  = Function('f', {x, u, r}, {xdot, L} ,...
     char('states', 'controls','reference'), char('ode', 'Mayer Term'));

% Evaluate a function numerically
x0  = [0;0;0;0];   % equilibrium point  
u0  = [0;0];       % equilibrium input
r0  = [0;0;0;0];   % equilibrium point
sol = f(x0,u0,r0);    % --> [0;0;0;0]
sol = full(sol);       
disp(sol);

%% ========================================================================
% Control discretization
% Time horizon
T  = 4;
M  = 4;
DT = 0.01;
N  = T/(DT*M);

X0p     = MX.sym('X0', nx);
U       = MX.sym('U' , nu);
R       = MX.sym('R', nx);
X       = X0p;
Q       = 0;

for j=1:M
    [k1, k1_q] = f(X            , U , R);
    [k2, k2_q] = f(X + DT/2 * k1, U , R);
    [k3, k3_q] = f(X + DT/2 * k2, U , R);
    [k4, k4_q] = f(X + DT   * k3, U , R);
    X = X + DT/6*(k1   + 2*k2   + 2*k3   + k4  );
    Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
end
F  = Function('F', {X0p, U , R}, {X, Q});

%% Start with an empty NLP ================================================
w      = {};      % solution array
w0     = [];      % initial guess array
lbw    = [];      % lower bound for w
ubw    = [];      % upper bound for w
J      = 0; 
g      = {};
lbg    = [];
ubg    = [];
r      = {};     % trajectory+x0 used as parameters

% input constraints
u_min  = [-0.8;-0.8];
u_max  = [ 0.8; 0.8];

% state constraints
x_min = [-inf; -inf; -inf; -inf];
x_max = [ inf;  inf;  inf;  inf];

% "Lift" initial conditions
X0p    = MX.sym('X0', nx);
r      = {r{:}, X0p};

% Formulate the NLP
Xk = X0p;
for k=0:N-1
    % New NLP variable for the control
    Uk  = MX.sym(['U_' num2str(k)],nu);
    w   = {w{:}, Uk};
    % bound on input   
    lbw = [lbw;  u_min];
    ubw = [ubw;  u_max];
    w0  = [w0 ;  zeros(nu,1)];

    % Integrate till the end of the interval    
    Rk  = MX.sym(['RX_' num2str(k)], nx);
    
    [Xk_end, Jk] = F(Xk, Uk, Rk);
    J = J + Jk;  
    
    % New NLP variable for state at end of interval
    Xk  = MX.sym(['X_' num2str(k+1)], nx);
    w   = {w{:}, Xk};
    lbw = [lbw; x_min];
    ubw = [ubw; x_max];
    w0  = [w0 ; zeros(nx,1)];
    
    % Add equality and Path constraints
    g   = {g{:}, Xk_end - Xk};
    lbg = [lbg; zeros(nx,1)];
    ubg = [ubg; zeros(nx,1)];
    % add reference
    r   = {r{:}, Rk};
end

%final condition (problem relaxed)
% lbw(end-nx+1:end) = x_min;
% ubw(end-nx+1:end) = x_max;

% create trajectory  and initial condition
x0p = [0;0.2;0;-0.1];
ref = zeros(nx,N);
ref(1,N/2:N) =   1;
ref(3,N/2:N) =   1;

% final condition (more strict problem)
lbw(end-nx+1:end) = ref(:,end);
ubw(end-nx+1:end) = ref(:,end);

ref = [x0p,ref];

% Create an NLP solver
% option IPOPT
% see: http://www.coin-or.org/Ipopt/documentation/node39.html
opts                             = struct;
opts.ipopt.max_iter              = 500;
opts.ipopt.linear_solver         = 'ma27';
%opts.ipopt.hessian_approximation = 'limited-memory';
opts.ipopt.hessian_approximation = 'exact';

%% Solve the NLP
% solve with QPoases
%qp = struct('x', vertcat(w{:}),'f',J,'g',vertcat(g{:}));
%solver = qpsol('solver', 'qpoases', qp);
% Solve with IPOPT
prob   = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}),'p',vertcat(r{:}));
solver = nlpsol('solver', 'ipopt', prob,opts);
sol    = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg,'p',reshape(ref,nx*(N+1),1));
w_opt  = full(sol.x);

T_opt       = w_opt(1:nx+nu:end);
F_opt       = w_opt(2:nx+nu:end);

y_opt       = w_opt(3:nx+nu:end);
dy_opt      = w_opt(4:nx+nu:end);
z_opt       = w_opt(5:nx+nu:end);
dz_opt      = w_opt(6:nx+nu:end);

figure;
time = linspace(0, T, N+1);
subplot(3,1,1);hold on;grid on;
plot(time,[ref(1,2:end),nan],'r-.');
plot(time,[x0p(1);y_opt]    ,'r');
plot(time,[ref(3,2:end),nan],'b-.');
plot(time,[x0p(3);z_opt]    ,'b');
legend('ry[t]','y[t]','rz[t]','z[t]');

subplot(3,1,2);hold on;grid on;
plot(time,[x0p(2);dy_opt]   ,'r');
plot(time,[x0p(4);dz_opt]   ,'b');
legend('dy[t]','dz[t]');

subplot(3,1,3);hold on;grid on;
stairs(time,[T_opt;nan],'r');
stairs(time,[F_opt;nan],'b');
legend('T[t]','F[t]');