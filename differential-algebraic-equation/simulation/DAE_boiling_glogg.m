%% Exercise 10 - Simulation on DAE
clear all;close all;clc;
import casadi.*

% define constant
Q = 1e-3;                       % eating power [W]
h = 40e3;                       % eating of evaporation [W/mol]
A1 = 8.1; B1 = 1730; C1 = 233;  % Antoine equation coefficients, water
A2 = 8.2; B2 = 1643; C2 = 230;  % Antoine equation coefficients, ethanol
p0 = 760;                        % air pressure in [mmHg]

% Declare variables
n = SX.sym('n',1);  % amount of glogg
c = SX.sym('c',1);  % concentration of ethanol
x = [n;c];
T = SX.sym('T',1);  % algebraic variable
z = T;

% Pressures
p1 = 10^(A1-B1/(C1+T));
p2 = 10^(A2-B2/(C2+T));
%% ODE
ndot = -Q/h;
cdot = -Q/h/n*(p2/p0 - c);
xdot = [ndot; cdot];
% Slgebraic equation
alg  = p1*(1-c) + p2*c - p0;

%% find initial temperature using T0 = 100 deg
% declare rootfinder
rfp  = Function('rfp', {z,x},{alg},char('algebraic state','differential state'),char('algebraic constraint'));                
rf   = rootfinder('rf','newton',rfp);
rf.printDimensions()

n0     = 250;
c0     = 0.2;
x0     = [n0; c0];
Tguess = 100;
% retrieve consisten initial condition
T0     = rf(Tguess,x0); 

display(T0)

%% Create an integrator object using SUNDIALS suite
tf   = 7200; %2h
prob = struct('x',x,'z',z,'ode', xdot, 'alg', alg);
opts = struct('tf',tf);

ivp_solver = casadi.integrator('ivp_solver','idas',prob,opts);
res        = ivp_solver('x0',x0,'z0',T0);
disp(res.xf);
disp(res.zf);
