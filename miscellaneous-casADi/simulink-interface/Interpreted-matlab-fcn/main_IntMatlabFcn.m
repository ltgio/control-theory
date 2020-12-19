%% run casadi function in simulink using interpreted MATLAB function
clc;clear all;close all;
import casadi.*

x = MX.sym('x',1);
y = MX.sym('y',1);
F = Function('F',{x},{x^2},{'x'},{'r'}); % 1-input | 1-output
G = Function('G',{x,y},{y*x^2},{'x','y'},{'r'}); % 2-input | 1-output

%% from command line
y  = full(F(1))
y1 = full(G(1,1))

%% Ode example
x = MX.sym('x',2);
u = MX.sym('u',1);
x1 = x(1);
x2 = x(2);

ode = [(1-x2^2)*x1-x2 + u; x1];
Fode = Function('ode',{x,u},{ode},char('x','u'),char('ode'));
Fode.expand();
x0 = [-1;0.5];
%Fode(x0,0);
%%
open('IntMatlabFcn_sim.slx')
