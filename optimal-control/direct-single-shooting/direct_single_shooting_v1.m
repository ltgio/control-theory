% Title:   Single Shooting 
% Version: Matlab 2014a/casadi3.0.0-rc3
% Author:  Marie Curie PhD student Giovanni Licitra
% Data:    03-03-2016

import casadi.*

T = 10;      % Time horizon
N = 20;      % number of control intervals
M = 4;       % RK4 steps per interval
dt = T/N/M;  

% Declare model variables
x1 = MX.sym('x1',1);
x2 = MX.sym('x2',1);
x  = [x1; x2];
u  = MX.sym('u',1);

% Model equations
xdot = [(1-x2^2)*x1 - x2 + u; 
         x1];

% Objective term
L = x1^2 + x2^2 + u^2;

% Continuous time dynamics
f  = Function('f', {x, u}, {xdot, L});
X0 = MX.sym('X0', 2);
U  = MX.sym('U' , 1);
X  = X0;
Q  = 0;

for j=1:M
    [k1, k1_q] = f(X            , U);
    [k2, k2_q] = f(X + dt/2 * k1, U);
    [k3, k3_q] = f(X + dt/2 * k2, U);
    [k4, k4_q] = f(X + dt   * k3, U);
    X = X + dt/6*(k1 +2*k2 +2*k3 +k4);
    Q = Q + dt/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
end
% F : x(k+1) = F(x(k),u(k))
F = Function('F', {X0, U}, {X, Q});

% Evaluate at a test point
[X_test, Q_test] = F([0.2; 0.3], 0.4);
display(X_test)
display(Q_test)

% Build nlp for Single Shooting
%Start with an empty NLP
w   = {}; % solution array
w0  = []; % initial guess array
lbw = []; % lower bound solution
ubw = []; % upper bound solution
J   = 0;  % index quadratic cost
g   = {}; % inequality array
lbg = []; % lower bound inequality array
ubg = []; % upper bound inequality array

% Formulate the NLP
Xk = [0 ; 1];
for k = 0:N-1
    % New NLP variable for the control
    Uk  = MX.sym(['U_' num2str(k)]);
    w   = {w{:}, Uk};
    w0  = [w0,  0];
    % bound on input  -1 <= u <= 1 
    lbw = [lbw, -1]; 
    ubw = [ubw,  1];
    % Integrate till the end of the interval
    [Xk, Jk] = F(Xk, Uk);
    J = J + Jk;
    
    % Add inequality constraint
    %            x(1)   x(2)
    g   = {g{:}, Xk(1), Xk(2)};
    lbg = [lbg; -0.25 ; -2 ]; % lower bound x
    ubg = [ubg;   inf ; inf]; % upper bound x
end

% Create an NLP solver
prob   = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
solver = nlpsol('solver', 'ipopt', prob);
% Solve the NLP
sol    = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);
w_opt  = full(sol.x);

%% Plot the solution
u_opt = w_opt;
x_opt = [0;1];
for k=0:N-1
    [Xk, Jk] = F(x_opt(:,end), u_opt(k+1));    
    x_opt = [x_opt, full(Xk)];
end
x1_opt = x_opt(1,:);
x2_opt = x_opt(2,:);
tgrid = linspace(0, T, N+1);
clf;
hold on
plot(tgrid, x1_opt, '--')
plot(tgrid, x2_opt, '-')
stairs(tgrid, [u_opt; nan], '-.')
xlabel('t')
legend('x1','x2','u')
