%  Tested on:
%  Matlab 2014a using casADi-MatlabR2014a-v.3.0.0-rc3
%  Matlab 2014b using casADi-MatlabR2014b-v.3.0.0-rc3
clear all;close all;clc;
import casadi.*
% Constants
N       = 40;
m_i     = 40.0/N;
D_i     = 70.0*N;
g0      = 9.81;
%zmin    = -inf  % unbounded
zmin    = 0.5; %ground
% Objective function
Vchain = 0;
% Variables
x      = {};
% Variable bounds
lbx    = [];
ubx    = [];
% Constraints
g      = {};
% Constraint bounds
lbg    = [];
ubg    = [];
% Loop over all chain elements
for i=0:N
   % Previous point
   if i>0
      y_prev = y_i;
      z_prev = z_i;
   end   
   % Create variables for the (y_i, z_i) coordinates
   y_i = SX.sym(['y_', num2str(i)]);
   z_i = SX.sym(['z_', num2str(i)]);
   % Add to the list of variables
   x = [x, {y_i, z_i}];
   if (i==0)
    lbx  =  [lbx;-2.; 1.];
    ubx  =  [ubx;-2.; 1.];
   elseif (i==N)
    lbx  = [lbx; 2.; 1.];
    ubx  = [ubx; 2.; 1.];
   else
    lbx  = [lbx;-inf; zmin];
    ubx  = [ubx; inf;  inf];
   end
   % Spring potential
   if (i>0)
      Vchain = Vchain + D_i/2*((y_prev-y_i)^2 + (z_prev-z_i)^2);
   end
   % Graviational potential
   Vchain =  Vchain + g0 * m_i * z_i;
   % Slanted ground constraints
   g   = [g, {z_i - 0.1*y_i}];
   lbg = [ lbg; 0.5 ];
   ubg = [ ubg; inf];
end

% Formulate QP
prob = struct('x',vertcat(x{:}),'f',Vchain,'g',vertcat(g{:}));
% Solve with IPOPT
solver = nlpsol('solver','ipopt',prob);
% Get the optimal solution
sol = solver('lbx',lbx, 'ubx',ubx, 'lbg',lbg, 'ubg',ubg);
% Retrieve the result
x_opt = full(sol.x);
Y0 = x_opt(1:2:end);
Z0 = x_opt(2:2:end);
 
% Plot the result
plot(Y0,Z0,'o-')
ys = linspace(0.,2.,100);
zs = zmin + 0.1*ys;
hold all;
plot(ys,zs,'black')
ys = linspace(-2.,0.,100);
zs = zmin*ones(100,1);
plot(ys,zs,'black')
xlabel('y [m]')
ylabel('z [m]')
title('Hanging chain QP')
grid on
legend('chain','z - 0.1y >= 0.5')
ylim([0.3,1 ])