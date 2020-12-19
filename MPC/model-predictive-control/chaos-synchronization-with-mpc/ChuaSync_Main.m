%  Author: Giovanni licitra  
%  date: 28-Jul-15           

function ChuaSync_Main              
clc;close all;
T = 10;
us = [0;0;0];                         % equilibrium input
p = [10;00/7];                        % parameters

%% Get equilibrium Points
x0 = [0;0;0;0;0;0];
%xs = fsolve(@(x) ode(0,x,us,p),x0)

xs = EquilibriumCR(p,us,x0);
%% get linear system
[A,B,C,D] = linearise(xs,us,p)
sys = ss(A,B,C,D);
[t,x_ode45] = ode45(@(t,x) ode(t,x,us,p),[0 T], x0);
figure(1);hold on;
plot(t,x_ode45);
end
function dx = ode(t,x,u,p)                 
    % NB: only constant input
    r = p(1);q = p(2);
    x1 = x(1);y1 = x(2);z1 = x(3);
    x2 = x(4);y2 = x(5);z2 = x(6);
    
    f_x1 = 2*x1^3 - x1/7;
    f_x2 = 2*x2^3 - x2/7;
    
    dx = [r*(y1-f_x1);
          x1-y1+z1;
          -q*y1;          
          r*(y2 - f_x2);
          x2 - y2 + z2;
          -q*y2;
          ];    
end

function [xs]= EquilibriumCR(p,us,x0);     
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

A = jacobian(dx,x);B = jacobian(dx,u);
C = jacobian(y1,x);D = jacobian(y1,u);

k1 = p(1);   k2 = p(2);
u1 = us(1);  u2 = us(2);                % equilibrium input
x1 = xs(1);  x2 = xs(2);                % equilibrium state

A = eval(A);B = eval(B);
C = eval(C);D = eval(D);
end