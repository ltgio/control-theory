% Version: Matlab 2014b
% Author:  Marie Curie PhD student Giovanni Licitra
% Data:    19-01-2016
% ChemicalReactor
% Equilibrium point
% forward simulation
% linearization

function ChemicalReactor_sym            
clc;close all;
T = 10;
k1 = 0.1;k2 = 0.5;
us = [1;-2];                         % equilibrium input
p  = [k1;k2];                        % parameters

%% Get equilibrium Points
x0 = [0.2;0.1];
xs = fsolve(@(x) ode(0,x,us,p),x0)

u0 = [1;0.5];
us = fsolve(@(u) ode(0,xs,u,p),u0)

xs = EquilibriumCR(p,us,x0);
%% get linear system
[A,B,C,D] = linearise(xs,us,p)
sys = ss(A,B,C,D);
[t,x_ode45] = ode45(@(t,x) ode(t,x,us,p),[0 T], x0);
figure(1);hold on;
plot(t,x_ode45);
end
function dx = ode(t,x,u,p)                 
    % Chemical reactor: Non linear Model pag.71 I sistemi lineari
    % NB: only constant input
    k1 = p(1);  k2 = p(2);
    x1 = x(1);  x2 = x(2);
    u1 = u(1);  u2 = u(2);

    dx = [u1-k1*x1;
          u1*u2/x1-u1*x2/x1-k2*x2];
end
function [xs]= EquilibriumCR(p,us,x0)     
% provide the equilibrium state xs
clc;close all;
k1 = 1;k2 = 1;
p  = [k1;k2];                        % parameters
% Get equilibrium Points
xs = fsolve(@(x) ode(0,x,us,p),x0);
end
function [A,B,C,D] = linearise(xs,us,p)    
% provide LTI system matrices around xs and us
syms k1 k2 x1 x2 u1 u2             % define parameters, states, controls
x = [x1;x2];u = [u1;u2];

dx = ode(0,x,u,p); 
y1 = x2/u2;

A = jacobian(dx,x)
B = jacobian(dx,u)
C = jacobian(y1,x);D = jacobian(y1,u);

k1 = p(1);   k2 = p(2);
u1 = us(1);  u2 = us(2);                % equilibrium input
x1 = xs(1);  x2 = xs(2);                % equilibrium state

A = eval(A);B = eval(B);
C = eval(C);D = eval(D);
end
