% Version: Matlab 2014b
% Author:  Marie Curie PhD student Giovanni Licitra
% Data:    19-01-2016
% ChemicalReactor
% Equilibrium point
% forward simulation
% linearization

function VTOL_fsolve          
clc;close all;
T = 10;

% Parameters ==============================================================
mass  = 3*10^4;               % massa veivolo [kg]
J     = 3*10^4;               % inerzia [kg*m^2]
l     = 4;                    % lunghezza alare [m]
alpha = pi/8;                 % angolo [rad]
g     = 9.81;                 % accelerazione gravitazionale
p     = [mass;J;l;alpha;g];   % parameters

%% Get equilibrium Points
u0 = [mass*g+100;102];                         % equilibrium input
x0  = [0.2;0.2;-0.1;0.5;0.6;0.8];
xeq = [0;0;0;0;0;0];

opts  = optimoptions('fsolve','Display','iter','Algorithm','Levenberg-Marquardt');
us = fsolve(@(u) ode(0,xeq,u ,p),u0,opts)

xs = fsolve(@(x) ode(0,x ,us,p),x0)

end
function dx = ode(t,x,u,p)                 
    mass  = p(1);
    J     = p(2);
    l     = p(3); 
    alpha = p(4);
    g     = p(5);
   
    y      = x(1);
    z      = x(2);
    theta  = x(3);
    dy     = x(4);
    dz     = x(5);
    dtheta = x(6);
     
    Taero = u(1);
    Faero = u(2);
        
    % define ode
  dx = [ dy                                                            ;...
         dz                                                            ;...
         dtheta                                                        ;...
        -Taero/mass*sin(theta) + 2*Faero/mass*sin(alpha)*cos(theta)    ;...
         Taero/mass*cos(theta) + 2*Faero/mass*sin(alpha)*sin(theta) - g;...
         2*l*Faero/J*cos(alpha)                                       ];    
end
