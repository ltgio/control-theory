%% main_ChaosSynch
clc;clear all;close all;
global x0 
x0 = [0.9365;-0.061;0.1889;0.02;0.05;0.04];  % initial state
T = 20;N = 2000;Ts = T/N;
input.Ts = Ts;input.nSteps = 2;

%% Discretize and linearize the dynamic system:
% nominal parameter: p = 10; q = 100/7;
x_lin = zeros(6,1); u_lin = zeros(3,1);
input.x = x_lin; input.u = u_lin;
%% Shaping Matrix for for Optimal Control
C = [1 0 0 -1  0  0;0 1 0  0 -1  0;0 0 1  0  0 -1]; 
Qd = 10*(C')*C;Rd = 0.1*eye(3);beta = 0.8;
%% Vertex 1
%ps = 10-1; qs = 100/7-1;  % Vertex1 
output = RK4_integrator( @ode_V1, input );
Ad_V1 = output.sensX;Bd_V1 = output.sensU;
[~,P_V1] = dlqr(sqrt(beta)*Ad_V1,sqrt(beta)*Bd_V1,Qd,Rd);
cost_V1 = max(svds(P_V1))
%% Vertex 2
%ps = 10-1; qs = 100/7+1;  % Vertex2 
output = RK4_integrator( @ode_V2, input );
Ad_V2 = output.sensX;Bd_V2 = output.sensU;
[~,P_V2] = dlqr(sqrt(beta)*Ad_V2,sqrt(beta)*Bd_V2,Qd,Rd);
cost_V2 = max(svds(P_V2))

%% Vertex 3
%ps = 10+1; qs = 100/7-1;  % Vertex3 
output = RK4_integrator( @ode_V3, input );
Ad_V3 = output.sensX;Bd_V3 = output.sensU;
[~,P_V3] = dlqr(sqrt(beta)*Ad_V3,sqrt(beta)*Bd_V3,Qd,Rd);
cost_V3 = max(svds(P_V3))

%% Vertex 4
%ps = 10+1; qs = 100/7+1;  % Vertex4 
output = RK4_integrator( @ode_V4, input );
Ad_V4 = output.sensX;Bd_V4 = output.sensU;
[~,P_V4] = dlqr(sqrt(beta)*Ad_V4,sqrt(beta)*Bd_V4,Qd,Rd);
cost_V4 = max(svds(P_V4))   

% Design Robust MPC
Ad = Ad_V4;Bd = Bd_V4;Pd = P_V4;

%% Design MPC using MPT3
mpt_init
umin =  0.2.*[-1;-1;-1];
umax =  0.2.*[1;1;1];
N = 20; % prediction horizon

% Prediction model (general template for linear systems)
model = LTISystem('A', sqrt(beta)*Ad, 'B', sqrt(beta)*Bd); % x(k+1) = A*x(k) + B*u(k)
model.u.min = umin;
model.u.max = umax;
model.u.penalty = QuadFunction(Rd); % quadratic penalty u'*R*u
model.x.penalty = QuadFunction(Qd); % quadratic penalty x'*Q*x

% the terminal penalty is optional and has to be enabled by the user
model.x.with('terminalPenalty');
model.x.terminalPenalty = QuadFunction(Pd);

% Construct MPC solution
mpc = MPCController(model, N);
[u, feasible, openloop] = mpc.evaluate(x0) % try

% Save data
% data.error = error;
% data.param = param;
% data.xout = xout;
% data.control = u;
% data.noyse = ny;
% data.info.bound = 'constraint controls umax = 0.1 umin = -0.1';
% data.info = 'chaos synch using two identical chuas circuit.Robust MPC finite horizon';
% data.info.param = 'parametri nello slave con incertezza p=10+-1, q=100/7+-1';
% data.info.x0 = 'condizioni iniziali xm = [0.02;0.05;-0.04] xs = [0.0002;0.0005;-0.0004]';
% data.info.noyse = 'rumone additivo sull uscita dell master mean=0, variance=0.001';
% data.info.benefit = 'controllo robusto,imposizione di vincoli. algoritmo decisamente piu veloce rispetto MPC-IH visto nel altro paper'
% 
