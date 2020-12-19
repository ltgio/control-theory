%% OCP for DAE using Direct Collocation
% An implementation of direct collocation
% efficient sparsity!!!

clc;clear all;close all;
import casadi.*

% Degree of interpolating polynomial
d  = 3; 
% Get collocation points
%tau_root = [0 collocation_points(d, 'legendre')];
tau_root = [0 collocation_points(d, 'radau')];
% Coefficients of the collocation equation
C = zeros(d+1,d+1);
% Coefficients of the continuity equation
D = zeros(d+1, 1);
% Coefficients of the quadrature function
B = zeros(d+1, 1);
% Construct polynomial basis
for j=1:d+1
  % Construct Lagrange polynomials to get the polynomial basis at the collocation point
  coeff = 1;
  for r=1:d+1
    if r ~= j
      coeff = conv(coeff, [1, -tau_root(r)]);
      coeff = coeff / (tau_root(j)-tau_root(r));
    end
  end
  % Evaluate the polynomial at the final time to get the coefficients of the continuity equation
  D(j) = polyval(coeff, 1.0);
  % Evaluate the time derivative of the polynomial at all collocation points to get the coefficients of the continuity equation
  pder = polyder(coeff);
  for r=1:d+1
    C(j,r) = polyval(pder, tau_root(r));
  end
  % Evaluate the integral of the polynomial to get the coefficients of the quadrature function
  pint = polyint(coeff);
  B(j) = polyval(pint, 1.0);
end

% Time horizon
T  = 10;

% Declare model variables
x1 = SX.sym('x1');
x2 = SX.sym('x2');
x  = [x1; x2];     % Differential states
z  = SX.sym('z');  % Algebraic variable
u  = SX.sym('u');  % Control Input

[nx,~] = size(x);
[nz,~] = size(z);
[nu,~] = size(u);

f_x = [(1-x2^2)*x1 - x2 + u; x1]; % Model equations
f_z = x1+x2-z;                    % algebraic equation
f_q = x1^2 + x2^2 + u^2;          % Lagrange cost term (quadrature)

% Continuous time dynamics
f = Function('f', {x,z,u}, {f_x,f_z,f_q});

% Control discretization
N = 20;  % number of control intervals
h = T/N; % Integration step 

% Start with an empty NLP
w   = {};
w0  = [];
lbw = [];
ubw = [];
J   = 0;
g   = {};
lbg = [];
ubg = [];

% initial condition
x0 = [0;1]; 

% input constraints
umin = [-1];
umax = [ 1];

% state constraints
xmin = [-0.25; -3];
xmax = [  inf;  3];

% algebraic constraints
zmin = [-2];
zmax = [ 2];

% "Lift" initial conditions [start nlp]
X0  = MX.sym('X0',nx);
w   = {w{:}, X0};
lbw = [lbw; x0];
ubw = [ubw; x0];
w0  = [w0 ; x0];

% Formulate the NLP
Xk = X0;
for k=0:N-1
    %% New NLP variable for the control
    Uk  = MX.sym(['U_' num2str(k)],nu);
    w   = {w{:}, Uk};
    lbw = [lbw; umin];
    ubw = [ubw; umax];
    w0  = [w0;  zeros(nu,1)];

    %% State at collocation points
    Xkj = {};
    for j=1:d
        Xkj{j} = MX.sym(['X_' num2str(k) '_' num2str(j)], nx);
        w = {w{:}, Xkj{j}};
        lbw = [lbw; xmin];
        ubw = [ubw; xmax];
        w0  = [w0 ; zeros(nx,1)];
    end
    
    %% algebraic states at collocation points
    Zkj = {};
    for j=1:d
        Zkj{j} = MX.sym(['Z_' num2str(k) '_' num2str(j)], nz);
        w   = {w{:}, Zkj{j}};
        lbw = [lbw; zmin];
        ubw = [ubw; zmax];
        w0  = [w0 ; zeros(nz,1)];
    end

    % Loop over collocation points
    Xk_end = D(1)*Xk;
    for j=1:d
       % Expression for the state derivative at the collocation point
       xp = C(1,j+1)*Xk;
       for r=1:d
           xp = xp + C(r+1,j+1)*Xkj{r};
       end
       
       %% Append collocation equations [continuity condition]
       [fx_j, fz_j, qj] = f(Xkj{j},Zkj{j},Uk);
       g   = {g{:}, h*fx_j - xp, fz_j};
       lbg = [lbg; zeros(nx+nz,1)];
       ubg = [ubg; zeros(nx+nz,1)];
       % Add contribution to the end state
       Xk_end = Xk_end + D(j+1)*Xkj{j};
       % Add contribution to quadrature function
       J = J + B(j+1)*qj*h;
    end

    %% New NLP variable for state at end of each interval
    Xk  = MX.sym(['X_' num2str(k+1)], nx);
    w   = {w{:}, Xk};
    lbw = [lbw; xmin];
    ubw = [ubw; xmax];
    w0  = [w0 ; zeros(nx,1)];

    % Add equality constraint
    g   = {g{:}, Xk_end-Xk};
    lbg = [lbg; zeros(nx,1)];
    ubg = [ubg; zeros(nx,1)];
end

% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
% option IPOPT
% see: http://www.coin-or.org/Ipopt/documentation/node39.html
opts                             = struct;
opts.ipopt.max_iter              = 200;
opts.ipopt.linear_solver         = 'ma27';
opts.ipopt.hessian_approximation = 'exact';

solver = nlpsol('solver', 'ipopt', prob,opts);

% Solve the NLP
sol   = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,'lbg', lbg, 'ubg', ubg);
w_opt = full(sol.x);

% Plot the solution
figure
%nw = nx+nu+2*d+1*d;
nw = nx*(d+1)+ d*nz + nu;

x1_opt = w_opt(1:nw:end);
x2_opt = w_opt(2:nw:end);
u_opt  = w_opt(3:nw:end);
z_opt  = w_opt(nw:nw:end);

%z_opt = [w_opt(10:nw:end),w_opt(11:nw:end),w_opt(12:nw:end)]';
%z_opt = reshape(z_opt,d*N,1);
%tgrid_collocation = linspace(0, T, d*N);
%stairs(tgrid_collocation, z_opt, '-.')

tgrid = linspace(0, T, N+1);
clf;
hold on
plot(tgrid, x1_opt, '--')
plot(tgrid, x2_opt, '-')
stairs(tgrid, [u_opt; nan], '-.')
stairs(tgrid, [z_opt; nan], '-.')
xlabel('t')
legend('x1','x2','u','z')

% Inspect Jacobian sparsity
g        = vertcat(g{:});
w        = vertcat(w{:});
Jacobian = jacobian(g, w);
figure(2)
spy(sparse(DM.ones(Jacobian.sparsity())))

% Inspect Hessian of the Lagrangian sparsity
Lambda     = MX.sym('lam', g.sparsity());
Lagrancian = J + dot(Lambda, g);
Hessian    = hessian(Lagrancian, w);
figure(3)
spy(sparse(DM.ones(Hessian.sparsity())))