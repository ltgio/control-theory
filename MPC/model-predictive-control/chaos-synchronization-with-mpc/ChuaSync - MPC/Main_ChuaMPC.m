%% main_ChaosSynch
clc;clear all;close all;
global x0
x0 = [0.02;0.05;-0.04;0.0002;0.0005;-0.0004];  % initial state
%% Nonlinear simulation using the RK4 integrator
T = 20;
N = 2000;
Ts = T/N;
input.Ts = Ts;
input.nSteps = 2;

%% Discretize and linearize the dynamic system:
x_lin = zeros(6,1); u_lin = zeros(3,1);
input.x = x_lin; input.u = u_lin;
output = RK4_integrator( @ode, input );

Ad = output.sensX;
Bd = output.sensU;

%% Design of the LQR controller using dlqr:
C = [1 0 0 -1  0  0;
     0 1 0  0 -1  0;
     0 0 1  0  0 -1]; 
 
Q = 10*C'*C;R = 0.1*eye(3);beta = 0.8;

[K,P] = dlqr(sqrt(beta)*Ad,sqrt(beta)*Bd,Q,R);

%% Design MPC using MPT3
mpt_init
umin =  0.1.*[-1;-1;-1];
umax =  0.1.*[1;1;1];
N = 20; % prediction horizon

% Prediction model (general template for linear systems)
model = LTISystem('A', sqrt(beta)*Ad, 'B', sqrt(beta)*Bd); % x(k+1) = A*x(k) + B*u(k)
model.u.min = umin;
model.u.max = umax;
model.u.penalty = QuadFunction(R); % quadratic penalty u'*R*u
model.x.penalty = QuadFunction(Q); % quadratic penalty x'*Q*x

%% Slew rate
%model.u.with('deltaMin')
%model.u.deltaMin = 0.2.*[-1;-1;-1];
%model.u.with('deltaMax')
%model.u.deltaMax =  0.2.*[1;1;1];

% the terminal penalty is optional and has to be enabled by the user
model.x.with('terminalPenalty');
model.x.terminalPenalty = QuadFunction(P);

% Construct MPC solution
mpc = MPCController(model, N);
[u, feasible, openloop] = mpc.evaluate(x0) % try 
% slew rate formulation
%[u, feasible, openloop] = mpc.evaluate(x0,'u.previous', [1;1;1])
% sim('Chua_MPC.slx')

% load('MPC_result.mat')
% t = xout.time';
% x = xout.data';
% visualizeChua(t,x)
%load('MPC_result_SR.mat')
%visualizeChua(xout.time',xout.data',u.data')