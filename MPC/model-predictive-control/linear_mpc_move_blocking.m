% Linear MPC: Linear MPC with move blocking feature
% Version: Matlab 2016b/casadi3.0.0
% Author:  Marie Curie PhD student Giovanni Licitra

%clear all;clc;close all;
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
L    = x'*x + u'*u; 

% Continuous time dynamics
f  = Function('f', {x, u}, {xdot, L} ,...
     char('states', 'controls'), char('ode', 'Mayer Term'));

% Evaluate a function numerically
x0  = [0;0;0;0];   % equilibrium point 
u0  = [0;0];       % equilibrium input
sol = f(x0,u0);    % --> [0;0;0;0]
sol = full(sol);       
disp(sol);

%% ========================================================================
% Control discretization
% Time horizon
T  = 10;
M  = 4;
DT = 0.01;
N  = T/(DT*M);
Nc = N/2;

X0 = MX.sym('X0', nx);
U  = MX.sym('U' , nu);
X  = X0;
Q  = 0;

for j=1:M
    [k1, k1_q] = f(X            , U);
    [k2, k2_q] = f(X + DT/2 * k1, U);
    [k3, k3_q] = f(X + DT/2 * k2, U);
    [k4, k4_q] = f(X + DT   * k3, U);
    X = X + DT/6*(k1   + 2*k2   + 2*k3   + k4  );
    Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
end
F  = Function('F', {X0, U}, {X, Q});

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
x0     = [0;-0.2;0;0.2];
X0     = MX.sym('X0', nx);
w      = {w{:}, X0};

lbw    = [lbw;  x0];
ubw    = [ubw;  x0];
w0     = [w0;   x0];

% input constraints
u_min  = [-20;-20];
u_max  = [ 20; 20];
% state constraints

x_min = [-inf; -1; -inf; -1];
x_max = [ inf;  1;  inf;  1];

% Formulate the NLP
Xk = X0;

iNc = 0;
for k=0:N-1
    if iNc<Nc
        % New NLP variable for the control
        Uk  = MX.sym(['U_' num2str(k)],nu);
        w   = {w{:}, Uk};
        % bound on input   
        lbw = [lbw;  u_min];
        ubw = [ubw;  u_max];
        w0  = [w0 ;  zeros(nu,1)];

        % Integrate till the end of the interval
        [Xk_end, Jk] = F(Xk, Uk);
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
        iNc = iNc + 1;
    else
        % Integrate till the end of the interval
        [Xk_end, Jk] = F(Xk, Uk);
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
        iNc = iNc + 1;
    end
end

% final condition
lbw(end-nx+1:end) = xref;
ubw(end-nx+1:end) = xref;

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

%% Plot the solution
y_opt_Nc  = w_opt(1:nx+nu:Nc*(nx+nu)-5);
dy_opt_Nc = w_opt(2:nx+nu:Nc*(nx+nu)-4);
z_opt_Nc  = w_opt(3:nx+nu:Nc*(nx+nu)-3);
dz_opt_Nc = w_opt(4:nx+nu:Nc*(nx+nu)-2);
Fy_opt    = w_opt(5:nx+nu:Nc*(nx+nu)-1);
Fz_opt    = w_opt(6:nx+nu:Nc*(nx+nu)-0);

w_opt(1:Nc*(nx+nu)) = [];

y_opt_N   = w_opt(1:nx:end);
dy_opt_N  = w_opt(2:nx:end);
z_opt_N   = w_opt(3:nx:end);
dz_opt_N  = w_opt(4:nx:end);

y_opt  = [y_opt_Nc ;y_opt_N];
dy_opt = [dy_opt_Nc;dy_opt_N];
z_opt  = [z_opt_Nc ;z_opt_N ];
dz_opt = [dz_opt_Nc;dz_opt_N];

figure;
time  = linspace(0,T,N+1)';
subplot(3,1,1);hold on;grid on;
plot(time,y_opt    ,'r');
plot(time,dy_opt   ,'b');
legend('y[t]','dy[t]');

subplot(3,1,2);hold on;grid on;
plot(time,z_opt  ,'r');
plot(time,dz_opt  ,'b');
legend('z[t]','dz[t]');

Fy_opt(end) = [];
Fz_opt(end) = [];

subplot(3,1,3);hold on;grid on;
stairs(time,[Fy_opt;Fy_opt(end).*ones(N-Nc+1,1);nan],'r');
stairs(time,[Fz_opt;Fz_opt(end).*ones(N-Nc+1,1);nan],'b');
legend('Fy[t]','Fz[t]');
