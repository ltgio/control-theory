% Code from CasADi 3 demo from Joris Gillis
% Link: https://www.youtube.com/watch?v=cOglZbSstjQ
% 1_Introduction 
% Title: Golf example - code for Matlab 2014b/casadi
% Author: PhD student Giovanni Licitra
% Data: 19-12-2015

clc;clear all;close all;
import casadi.*                  % Load CasADi 

%% Model Parameters
m   = 0.04593;      % [kg]
g   = 9.81;         % [m/s^2]
rho = 1.225;        % [kg/m^3]
r   = 0.02135;      % [m]
A   = pi*r^2;       % [m^2]
CD  = 0.2;          % [-]  
c   = 0.5*rho*A*CD; % friction

% Scalar expression graphs (SX)
p = SX.sym('p',2);  % symbolic vector [2,1] position 
v = SX.sym('v',2);  % symbolic vector [2,1] velocity

states = [p;v]; 
speed  = norm_2(v);

% Right Hand Side ODE
rhs    = [v;-c*speed*v/m];
rhs(4) = rhs(4)- g;       % add gravity in y direction

% some useful function
J = jacobian(rhs,states)
size(J)
J.sparsity().spy()        % check sparsity

% construct function
f = Function('f',{states},{rhs});
f.printDimensions();

% numerical evaluation
x_in  = [0;0;5;5];    % [x=0;y=0;vx=5;vy=5] input is cell array type
x_out = f(x_in);      % output is cell array type as well
class(x_out)          % which class?
x_out = full(x_out);  % convert in double
class(x_out)

theta = SX.sym('theta');               % symbolic variable
f_in  = [0;0;cos(theta);sin(theta)];   % [x=0;y=0;vx=theta;vy=theta] input is cell array type
f_out = f(f_in)                        % output is cell array type as well

J = jacobian(f_out,theta)
J.sparsity().spy()

%% Input/output schemes: give a bit of standard notation
F = Function('f',{states},{rhs},char('states'),char('rhs'));
F.printDimensions();
f_out = F([0;0;5;5])  % feed the function F with [0;0;5;5]
                    
%% Task 1: Simulate the golfball flight ===================================
%  Integration example
%  ti = 0 [s]
%  tf = 5 [s]
%  v0 = [vx;vy]=[35;30] [m/s]
ti = 0; tf = 5;
x0 = [0.0;0.0;35.0;30.0]; % initial condition
h = 0.1;                  % step size [s]
t = [ti:h:tf-h]';         % time array
Xeuler = {};              % define empty cell array
xk = x0;                  % current state

tic
for k=1:length(t)
    F_out = full(F(xk));        % comput  f(x[k])
    xk = xk+h*F_out;            % compute x[k+1]
    Xeuler = {Xeuler{:} xk};    % append xk to Xeuler
end
CPUtimeEuler = toc

class(Xeuler)
Xeuler = [Xeuler{:}];           % convert from cell to euler 
Xeuler = full(Xeuler');         % retrieve simulation in double matrix type

figure;
subplot(1,2,1);plot(Xeuler(:,1),Xeuler(:,2),'rx');grid on;
               xlabel('x[t]');ylabel('y[t]');title('ball trajectory');
subplot(1,2,2);plot(Xeuler(:,3),Xeuler(:,4),'gx');grid on;
               xlabel('v_{x}[t]');ylabel('v_{y}[t]');title('speed ball');