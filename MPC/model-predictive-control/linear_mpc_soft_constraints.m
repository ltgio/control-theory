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

s_y   = SX.sym('s_y' ,1); 
s_vy  = SX.sym('s_vy',1);
s_z   = SX.sym('s_z' ,1);
s_vz  = SX.sym('s_vz',1);

sx  = [s_y;s_vy;s_z;s_vz];

Fy = SX.sym('Fy',1);
Fz = SX.sym('Fz',1);
u  = [Fy;Fz];

s_Fy = SX.sym('s_Fy',1);
s_Fz = SX.sym('s_Fz',1);
su  = [s_Fy;s_Fz];

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
%xref = [ 1; 0;-1; 0];
Qsx  = 100*eye(nx);
Qsu  = 100*eye(nu);

L    = x'*x + u'*u + sx'*Qsx*sx + su'*Qsu*su; 

% Continuous time dynamics
f  = Function('f', {x, u, sx, su}, {xdot, L} ,...
     char('states', 'controls'), char('ode', 'Mayer Term'));

% Evaluate a function numerically
x0  = [0;0;0;0];        % equilibrium point 
u0  = [0;0];            % equilibrium input
sx0 = [0;0;0;0];        % equilibrium point 
su0 = [0;0];            % equilibrium input
sol = f(x0,u0,sx0,su0); % --> [0;0;0;0]
sol = full(sol);       
disp(sol);

%% ========================================================================
% Control discretization
% Time horizon
T       = 10;
N       = 40; % number of control intervals
M       = 4;  % RK4 steps per interval
DT      = T/N/M;
X0      = MX.sym('X0' , nx);
U       = MX.sym('U'  , nu);
SX0     = MX.sym('SX0', nx);
SU      = MX.sym('SU' , nu);

X       = X0;
Q       = 0;

for j=1:M
    [k1, k1_q] = f(X            , U, SX0, SU);
    [k2, k2_q] = f(X + DT/2 * k1, U, SX0, SU);
    [k3, k3_q] = f(X + DT/2 * k2, U, SX0, SU);
    [k4, k4_q] = f(X + DT   * k3, U, SX0, SU);
    X = X + DT/6*(k1   + 2*k2   + 2*k3   + k4  );
    Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
end
F  = Function('F', {X0, U, SX0, SU}, {X, Q});

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
x0     = [0;0.2;0;-0.1];
s0     = zeros(4,1);

X0     = MX.sym( 'X0', nx);
SX0    = MX.sym('SX0', nx);

w      = {w{:}, [X0;SX0]};

lbw    = [lbw;  [x0;zeros(nx,1)]];
ubw    = [ubw;  [x0;zeros(nx,1)]];
w0     = [w0;   [x0;zeros(nx,1)]];

% input constraints
u_min  = [-0.8;-0.8];
u_max  = [ 0.8; 0.8];
% state constraints

x_min = [-10; -1; -10; -1];
x_max = [ 10;  1;  10;  1];

% path constraints
Vmin = zeros(nx+nu,1);
Vmax = inf.*ones(nx+nu,1);

% Formulate the NLP
Xk = X0;
for k=0:N-1
    S_Uk  = MX.sym(['SU_' num2str(k)],nu);
    S_Xk  = MX.sym(['SX_' num2str(k+1)],nx);
    
    % New NLP variable for the control
    Uk    = MX.sym(['U_'  num2str(k)],nu);
    
    w   = {w{:}, [Uk;S_Uk]};
    % bound on input   
    lbw = [lbw;  [u_min;zeros(nu,1)]];
    ubw = [ubw;  [u_max;inf.*ones(nu,1)]];
    w0  = [w0 ;  zeros(2*nu,1)];

    % Integrate till the end of the interval
    [Xk_end, Jk] = F(Xk, Uk, S_Xk, S_Uk);
    J = J + Jk; 
    
    % New NLP variable for state at end of interval
    Xk  = MX.sym(['X_' num2str(k+1)], nx);
    w   = {w{:}, [Xk;S_Xk]};
    lbw = [lbw; [x_min;zeros(nx,1)]];
    ubw = [ubw; [x_max;inf.*ones(nx,1)]];
    w0  = [w0 ; zeros(2*nx,1)];
    
    % define path constrains
    V    = [Uk + S_Uk; Xk + S_Xk]; 
    % Add equality and Path constraints
    g   = {g{:}, Xk_end - Xk , V};
    lbg = [lbg; zeros(nx,1)  ; Vmin];
    ubg = [ubg; zeros(nx,1)  ; Vmax];

end

% final path constraint
V   = [Uk + S_Uk; Xk + S_Xk];   
g   = {g{:}, V};
lbg = [lbg; Vmin];
ubg = [ubg; Vmax];

% final condition
% lbw(end-nx+1:end) = [xref;zeros(nx,1)];
% ubw(end-nx+1:end) = [xref;inf.*ones(nx,1)];

%% Solve the NLP
% solve with QPoases
%qp = struct('x', vertcat(w{:}),'f',J,'g',vertcat(g{:}));
%solver = qpsol('solver', 'qpoases', qp);
mySolver = 'IPOPT';
%mySolver = 'sqpmethod';

% Solve with IPOPT
switch mySolver
    case 'IPOPT'
        opts                             = struct;
        opts.ipopt.max_iter              = 500;
        opts.ipopt.linear_solver         = 'ma27';
        opts.ipopt.hessian_approximation = 'limited-memory';
        opts.ipopt.hessian_approximation = 'exact';
        prob   = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
        solver = nlpsol('solver', 'ipopt', prob,opts);
    case 'sqpmethod'
        opts.qpsol = 'qpoases';
        opts.qpsol_options.printLevel = 'none';
        prob    = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
        solver  = nlpsol('solver',mySolver,prob,opts);
end

sol    = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);
w_opt  = full(sol.x);

% Plot the solution
y_opt       = w_opt(1:nx+nu:end);
dy_opt      = w_opt(2:nx+nu:end);
z_opt       = w_opt(3:nx+nu:end);
dz_opt      = w_opt(4:nx+nu:end);
V_opt       = (dy_opt.^2+dz_opt.^2);

T_opt       = w_opt(5:nx+nu:end);
F_opt       = w_opt(6:nx+nu:end);

figure;
time = linspace(0, T, N+1);
subplot(3,2,1);hold on;grid on;
plot(time,y_opt    ,'r');
plot(time,z_opt    ,'b');
legend('y[t]','z[t]');

subplot(3,2,2);hold on;grid on;
plot(y_opt,z_opt    ,'m');
legend('(y[t],z[t])');

subplot(3,2,3);hold on;grid on;
plot(time,dy_opt    ,'r');
plot(time,dz_opt    ,'b');
legend('dy[t]','dz[t]');

subplot(3,2,4);hold on;grid on;
plot(time,V_opt    ,'g');
legend('V[t]');

subplot(3,2,5);hold on;grid on;
stairs(time,[T_opt;nan],'r');
stairs(time,[F_opt;nan],'b');
legend('T[t]','F[t]');


% 
% % Inspect Jacobian sparsity
% sol.g        = vertcat(g{:});
% sol.w        = vertcat(w{:});
% sol.Jacobian = jacobian(sol.g, sol.w);
% figure
% spy(sparse(DM.ones(sol.Jacobian.sparsity())))
% 
% % Inspect Hessian of the Lagrangian sparsity
% sol.Lambda     = MX.sym('lam', sol.g.sparsity());
% sol.Lagrancian = sol.f + dot(sol.Lambda, sol.g);
% sol.Hessian    = hessian(sol.Lagrancian, sol.w);
% figure;
% spy(sparse(DM.ones(sol.Hessian.sparsity())))