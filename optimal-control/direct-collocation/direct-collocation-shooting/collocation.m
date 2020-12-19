function F = collocation(dae, tf, nsteps)
% Joel Andersson, joel@casadi.org, 2016
import casadi.*
% Degree of interpolating polynomial
d = 3;
% Get collocation points
tau_root = [0 collocation_points(d, 'legendre')];
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
% Continuous time dynamics
f      = Function('f', {dae.x, dae.p}, {dae.ode, dae.quad});
% Variables for one finite element
X0     = MX.sym('X0', dae.x.sparsity());
P      = MX.sym('P', dae.p.sparsity());
X      = {};
for k = 1:d
    X{k} = MX.sym(['X' num2str(k)], dae.x.sparsity());
end
% Collocation equations and quadrature
g      = {};
qf     = 0;
xf     = D(1)*X0;
h      = tf/nsteps;
for j  = 1:d
  % Expression for the state derivative at the collocation point
  xp   = C(1,j+1)*X0;
  for r = 1:d
    xp  = xp + C(r+1,j+1)*X{r};
  end      
  % Append collocation equations
  [fj, qj] = f(X{j},P);
  g{j}     = h*fj - xp;
  % Add contribution to the end state
  xf       = xf + D(j+1)*X{j};
  % Add contribution to quadrature function
  qf       = qf + B(j+1)*qj*h;
end
% Rootfinder function, implicitly defines X and Q as a function of X0 and U
rfp        = Function('rfp', {vertcat(X{:}), X0, P},...
                             {vertcat(g{:}), xf, qf});                  
rf         = rootfinder('rf', 'newton', rfp);
% Discrete time dynamics
XF = X0;
QF = 0;
for j=1:nsteps
    [~, XF, Qj] = rf(repmat(XF, d, 1), XF, P);
    QF = QF + Qj;
end
F  = Function('F', {X0, P}, {XF, QF}, ...
     char('x0', 'p'), char('xf', 'qf'));
end