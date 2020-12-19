%% Plane and Cart Model using DAE formulation [2D]
% Version: Matlab 2014a
% Author:  Marie Curie PhD student koenemann & Giovanni Licitra
% Data:    11-02-2016

clc;clear;close all;
%% Paramters ==============================================================
p                          = struct;
p.massPlane                = 2;
p.AspectRatio              = 10;
p.airDensity               = 1.23; 
p.gravity                  = 9.81; 
p.wingArea                 = 0.5; 
p.massCart                 = 1;
p.aeroDragCoefficientPlane = 0.04;                                  % for StreamLine Body
p.aeroDragCoefficientCart  = 0.6;                                   % long cylinder      (static value)   Df = 0.82
p.rollDragCoefficientCart  = 0.06;                                  % Tire - wet Asphalt (dynamic value) Df = 0.6  
p.AreaCart                 = 0.1^2;
%% Define Model ===========================================================
u                          = 4*pi/180;                              % control is angle of attack
dae                        = @(t,x,dx)CartPlaneModel(t,x,dx,u,p);   % define DAE function
%% Compute Initial Condition ==============================================
x0_guess                   = [0;-50;0;10;0;10;0];                   %  x(t=0) guess
dx0_guess                  = [0;0;0;0;0;0;0];                       % dx(t=0) guess
[x0,dx0]                   = decic(dae,0,x0_guess,[],dx0_guess,[]); % Compute consistent initial conditions for ODE15I
%% Comput IVP by ode15i ===================================================
ts                         = 0.01;
Toss                       = 20;
tspan                      = [0:ts:Toss]';
opt.RelTol                 = 1e-12;
[tsim,Xsim]                = ode15i(dae,tspan,x0,dx0,opt);
%% Visualization and Plots ================================================ 
VisualizePlaneCart_2014b(tsim,Xsim)

