% Title:   Multiple Shooting 
% Version: Matlab 2014b/casadi3.0.0
% Author:  Marie Curie PhD student Giovanni Licitra
% Data:    04-03-2016
import casadi.*

% Time horizon
T    = 10;
% Declare model variables
x1   = MX.sym('x1');
x2   = MX.sym('x2');
x    = [x1; x2];
u    = MX.sym('u');

nx = 2;
nu = 1;

% Model equations
xdot = [(1-x2^2)*x1 - x2 + u; x1];
% Objective term
L    = x1^2 + x2^2 + u^2;
% Continuous time dynamics
f    = Function('f', {x, u}, {xdot, L});
% Control discretization
N    = 20; % number of control intervals
M    = 4;  % RK4 steps per interval
DT   = T/N/M;
X0   = MX.sym('X0', nx);
U    = MX.sym('U' , nu);
X    = X0;
Q    = 0;
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
w      = {};
w0     = [];
lbw    = [];
ubw    = [];
J      = 0;
g      = {};
lbg    = [];
ubg    = [];

% "Lift" initial conditions
X0     = MX.sym('X0', nx);
w      = {w{:}, X0};
lbw    = [lbw; 0; 1];
ubw    = [ubw; 0; 1];
w0     = [w0;  0; 1];

% Formulate the NLP
Xk = X0;
for k=0:N-1
    % New NLP variable for the control
    Uk  = MX.sym(['U_' num2str(k)]);
    w   = {w{:}, Uk};
    lbw = [lbw; -1];
    ubw = [ubw;  1];
    w0  = [w0;   0];

    % Integrate till the end of the interval
    [Xk_end, Jk] = F(Xk, Uk);
    J = J + Jk;

    % New NLP variable for state at end of interval
    Xk  = MX.sym(['X_' num2str(k+1)], nx);
    w   = {w{:}, Xk};
    lbw = [lbw; -0.25; -inf];
    ubw = [ubw;   inf;  inf];
    w0  = [w0;      0;    0];
        
    % Add equality constraint
    g   = {g{:}, Xk_end-Xk};
    lbg = [lbg; 0; 0];
    ubg = [ubg; 0; 0];
end

% Create an NLP solver
prob   = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
% option IPOPT
% see: http://www.coin-or.org/Ipopt/documentation/node39.html
opts                = struct;
opts.ipopt.max_iter = 20;
% Solve the NLP
solver = nlpsol('solver', 'ipopt', prob,opts);
sol    = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);
w_opt  = full(sol.x);

% Plot the solution
x1_opt = w_opt(1:3:end);
x2_opt = w_opt(2:3:end);
u_opt = w_opt(3:3:end);
tgrid = linspace(0, T, N+1);
clf;
hold on
plot(tgrid, x1_opt, 'r')
plot(tgrid, x2_opt, 'b')
stairs(tgrid, [u_opt; nan], 'g')
xlabel('t')
legend('x1','x2','u')