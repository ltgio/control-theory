%% main_ChaosSynch
clc;clear all;close all;
global x0
x0 = [0.02;0.05;0.04;0.0;1.0;0.0];  % initial state
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
 
Q = 1000*C'*C;R = 0.001*eye(3);beta = 0.5;

[K,P,E] = dlqr(sqrt(beta)*Ad,sqrt(beta)*Bd,Q,R);

%% Design MPC using MPT3
mpt_init
umin =  10.*[-1;-1;-1];
umax =  10.*[1;1;1];
N = 20; % prediction horizon

% Prediction model (general template for linear systems)
model = LTISystem('A', sqrt(beta)*Ad, 'B', sqrt(beta)*Bd); % x(k+1) = A*x(k) + B*u(k)
model.u.min = umin;
model.u.max = umax;
model.u.penalty = QuadFunction(R); % quadratic penalty u'*R*u
model.x.penalty = QuadFunction(Q); % quadratic penalty x'*Q*x

% the terminal penalty is optional and has to be enabled by the user
model.x.with('terminalPenalty');
model.x.terminalPenalty = QuadFunction(P);

% Construct MPC solution
mpc = MPCController(model, N);
[u, feasible, openloop] = mpc.evaluate(x0) % try 

%load('MPC_result.mat')
%visualizeChua(xout.time',xout.data',u.data')

data.info.info = 'master: chua poly, slave: Lorenz system';
data.info.control = 'bound controls +-10';
data.info.parameters = 'both master and slave have uncertainty parameters'
data.info.noise = 'noise in output of the master mean = 0 var = 0.001'
data.control = u;
data.error = error;
data.states = xout;
data.param_master = param_master;
data.param_slave = param_slave;




