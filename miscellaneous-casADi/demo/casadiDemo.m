%% casadi Demo 
%  Tested on:
%  Matlab 2014a using casADi-MatlabR2014a-v.3.0.0-rc3
%  Matlab 2014b using casADi-MatlabR2014b-v.3.0.0-rc3

clc;clear all;close all;
%  Load all CasADi symbols
import casadi.*
%% Scalar-atomic symbolic expression type (SX)
% Symbolic primitives
x  = SX.sym('x');
y  = SX.sym('y');
% Construct expressions
z  = x*sin(x+y)
% Expressions may have common subexpressions
z2 = z + cos(z)
% Some simplifications (but not much)
z3 = x*y/x - y
%% Everything is a matrix
A  = SX.sym('A',3,3)
B  = SX.sym('B',3)
z4 = A.*A             % Elementwise product, in Python "*"
z5 = A*B              % Matrix product, in Python "mtimes"
z6 = trace(A)         % Trace
z7 = norm_F(A)
z8 = A(3, :)          % Slicing: In Python A[2,:]
%% In fact, everything is a _sparse_ matrix
I = SX.eye(3);
Ak = kron(I,A)
%% Algorithmic differentiation (AD)
t   = SX.sym('t');       % time
u   = SX.sym('u');       % control
p   = SX.sym('p');
q   = SX.sym('q');
c   = SX.sym('c');
x   = [p;q;c];           % state
ode = [(1 - q^2)*p - q + u; ... 
       p;                   ... 
       p^2+q^2+u^2        ];
J   = jacobian(ode, x);
% Gradient of a scalar expression
g   = gradient(ode(3), p)
% Hessian of a scalar expression
H   = hessian(ode(3), x)
% Jacobian-times-vector product (forward mode AD)
v   = SX.sym('v', 3)
Jv  = jtimes(ode, x, v)
% Transposed Jacobian-times-vector product (reverse mode AD)
JTv = jtimes(ode, x, v, true)
%% CasADi functions
x   = SX.sym('x');
y   = SX.sym('y');
z   = x*sin(x+y);
% Multiple matrix-valued inputs, multiple matrix-valued outputs
f   = Function('f', {x,y}, {z});
% Or better, with names for inputs and outputs
f   = Function('f', {x,y}, {z}, char('x', 'y'), char('z'));
% Evaluate a function numerically
sol = f(1.2 , 3.4); % output DM(-1.19243)
sol = full(sol);    % get double  -1.1924
disp(sol);

% Evaluate a function symbolically
sol = f(1.2,x+y);
disp(sol);

% Other things, generate C code (with MEX gateway):
opts = struct('mex', true);
f.generate('f.c', opts);

%% Matrix expression graphs (use when in doubt)
A  = MX.sym('A',3,3);
B  = MX.sym('B',3);
z4 = A.*A                            % Elementwise product, in Python "*"
z5 = A*B                             % Matrix product, in Python "mtimes"
z6 = trace(A)                        % Trace
z7 = norm_F(A)
z8 = A(3, :)                         % Slicing: In Python A[2,:]
I = MX.eye(3);
Ak = kron(I,A)