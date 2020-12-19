% Code from CasADi 3 demo from Joris Gillis
% Link: https://www.youtube.com/watch?v=cOglZbSstjQ
% 1_Introduction 
% Title: Golf example - code for Matlab 2014b/casadi
% Author: PhD student Giovanni Licitra
% Data: 19-12-2015
clc;clear all;close all;
import casadi.*                  % Load CasADi 
tic
%% Model Parameters
m = 0.04593;      % [kg]
g = 9.81;         % [m/s^2]
rho = 1.225;      % [kg/m^3]
r = 0.02135;      % [m]
A = pi*r^2;       % [m^2]
CD = 0.2;      
c = 0.5*rho*A*CD; % friction

% Matrix expression type graphs (MX)
% MX type is optimized for memory usage

p = MX.sym('p',2);  % symbolic vector [2,1] position 
v = MX.sym('v',2);  % symbolic vector [2,1] velocity

states = [p;v]; 
speed = norm_2(v);

% Right Hand Side ODE
rhs = [v;...
       -c*speed*v/m];
rhs(4) = rhs(4)- g;    % add gravity in y direction

% some useful function
J = jacobian(rhs,states)
size(J)
J.sparsity().spy()

% construct function
f = Function('f',{states},{rhs});
f.printDimensions();

%% numerical evaluation
f_in  = [0;0;5;5];    % [x=0;y=0;vx=5;vy=5] input is cell array type
f_out = f(f_in)       % output is cell array type as well

type       = class(f_out)
f_out_cell = full(f_out)  % output in cell array type
class(f_out)
size = size(f_out)
f_out_matrix = full(f_out)  % conversion from cell array to matlab matrix
class(f_out_matrix)

CPUtime = toc

%% Symbolic evaluation of functions
theta = MX.sym('theta');             % symbolic variable
f_in  = [0;0;cos(theta);sin(theta)]; % [x=0;y=0;vx=theta;vy=theta] input is cell array type
f_out = f(f_in);                     % output is cell array type as well
f_out_cell = f_out;                  % output in cell array type
class(f_out)

J = jacobian(f_out,theta)
J.sparsity().spy()

%% Input/output schemes: give a bit of standard notation
F = Function('f',{states},{rhs},char('states'),char('rhs'));
F.printDimensions();

f_out = F([0;0;5;5])  % feed the function F with [0;0;5;5]
                    
% Task 1: Simulate the golfball flight ===================================
%%  Integration example: constant step size h
%  ti = 0 [s]
%  tf = 5 [s]
%  v0 = [vx;vy]=[35;30] [m/s]
ti = 0; 
tf = 5;
x0 = [0.0;0.0;35.0;30.0]; % initial condition
h  = 0.1;                 % step size [s]
t  = [ti:h:tf-h]';        % time array
N  = length(t);
Xeuler = {}               % define empty cell array
xk = x0;                  % current state

tic
for k=1:N
    F_out  = full(F(xk));       % compute f(x[k])
    xk     = xk+h*(F_out);      % compute x[k+1]
    Xeuler = {Xeuler{:} xk};    % append xk to Xeuler
end
CPUtimeEuler = toc

class(Xeuler)
Xeuler = [Xeuler{:}];      
Xeuler = full(Xeuler');         % retrieve simulation in double matrix type

%% Create an integrator object using SUNDIALS suite
%% adaptative stepsize integrator
opts  = struct('tf',h);
prob  = struct('x',states,'ode',rhs);
I = casadi.integrator('Integrator','cvodes',prob,opts); % integrator type
I.printDimensions();

Xcvodes = {};

xk = x0;
tic
for k=1:N
    I_out = I('x0',xk);          % compute  f(x[k])
    xk = I_out.xf;               % compute x[k+1]
    Xcvodes = {Xcvodes{:} xk};   % append xk to Xcvodes
end
CPUtimeCvodes = toc
Xcvodes = [Xcvodes{:}];
Xcvodes = full(Xcvodes');

figure;
subplot(1,2,1);plot(Xeuler(:,1) ,Xeuler(:,2) ,'rx');grid on;hold on;
               plot(Xcvodes(:,1),Xcvodes(:,2),'bo');legend('euler','cvodes');
               xlabel('x[t]');ylabel('y[t]');title('ball trajectory');
subplot(1,2,2);plot(Xeuler(:,3) ,Xeuler(:,4) ,'gx');grid on;hold on;
               plot(Xcvodes(:,3),Xcvodes(:,4),'bo');legend('euler','cvodes');
               xlabel('v_{x}[t]');ylabel('v_{y}[t]');title('speed ball');
               
%% when the horizon of Integration start to be really high e.g. s
%  system identification. For such case you can use the "mapaccum" functionality 

sim = I.mapaccum('mapaccum',N); % N is the number of steps
sim.printDimensions()

tic
sim_out = sim('x0',x0);
CPUtimeCodes = toc
Xcodes = sim_out.xf;       % retrieve integration
Xcodes = full(Xcodes');    % convert in [N,nx] double matrix

%% Task 3 =================================================================
%% Given an initial speed v and angle theta, how far can we shoot?
% such question involve an Optimal Control Problem using "time trasformation"
% let t = tau*T, with tau = 0,...,1
% dx   
% -- = f(x) for t = 0,...,T
% dt
%
% dx   
% -- = T*f(x) for tau = 0,...,1
% dt

% step 1: construct new DAE function
% 'p' stand for parameter i.e. T is kept constant over the integration
% interval

T   = MX.sym('T');
% dyn = Function('dyn',{states,T},{T*rhs},char('x','T'),char('T*rhs')); 
% dyn_fast = dyn.expand(); % speed up computation

opts  = struct('tf',h);
prob  = struct('x',states,'p',T,'ode',T*rhs);
fly = casadi.integrator('fly','cvodes',prob); % integrator type


% step 2: construct integrator
%fly = casadi.integrator('fly','cvodes',dyn_fast);
fly.printDimensions()

% optional: numerical evalutation
fly_out = fly('x0',[0;0;35;30],'p',5);                   % compute integration
xf_T = fly_out.xf;
xf_T = full(xf_T')

v0     = 35;  % [m/s]
theta0 = 0.7; % [rad]

x0      = [0;0;cos(theta0)*v0;sin(theta0)*v0];
fly_out = fly('x0',x0,'p',5);     
xf_T    = fly_out.xf;                      % state at time T = 5 [s]
xf_T    = full(xf_T') % --> [105.5651  py = -17.0610   16.5065  -25.1114]


% step 3: find the amount of time that we need for reach the ground
% Thus we require py(T) = 0. In effect, we are solvin an implicit fuction
% g(q) = 0 for q and we can solve it using Newton Method
% Newton scheme:
%               / dg        \-1
% q[i+1] = q[i]-| -- (q[i]) |   * g(q[i])  
%               \ dq        /

Jfly = fly.jacobian('p','xf'); % jabobian of the end=state respect p
Jfly.printDimensions()

qi   = 5;   % initial guess T = 5;
iter = 5;   % # of iteration
%% METHOD A: Manual Newton Method 
disp ('-----Newton Iteration-----');
for i = 1:iter
    J_out = Jfly('x0',x0,'p',qi);
    g     = J_out.xf(2);
    dgdq  = J_out.dxf_dp(2);
    qi    = qi - solve(dgdq,g);
    fprintf('%d [iter] %.3f [s] %.3f [m] \n', i, full(qi), full(J_out.xf(1)));
end
Tpy = full(qi); 
fprintf('The golf ball hit the ground at %.3f T = ', Tpy);

%% METHOD B: built-in Newton Method 
v     = MX.sym('v');
theta = MX.sym('theta');
T     = MX.sym('T');

% construct n-state symbolicly depended on velocity,angle,time flight
x0 = [0;0;v*cos(theta);v*sin(theta)]; 
fly_out = fly('x0',x0,'p',T);
xf = fly_out.xf;
% define residual function composed by list of input and list of output
% residual: (T,initial speed,angle) -> (y position, x position)
% initial speed,angle are parameters
% x position is a extra output: helper
residual = Function('residual',{T,v,theta},{xf(2),xf});
residual.printDimensions()
% evaluate residual (T=5,initial speed=35,angle=0.7) -> (-17.061, 105.565)
residual(5,35,0.7); 
% once we have the residual function we can solve it symbolically
shoot = rootfinder('shoot','newton',residual); % build Implicit Function
% Remark: shoot is just a label
shoot.printDimensions();
shoot(5,35,0.7); % solve py(T=5)=0 given v0 = 35, theta = 0.7

%% Plot how does the shooting depend on velocity and tetha
nv     = 10;
ntheta = 12;
ntot   = nv*ntheta;
[vgrid,thetagrid] = meshgrid(linspace(1,50,nv),linspace(10,80,ntheta));

% use map to avoif a for loop
vflat         = reshape(vgrid,1,ntot);
thetaflat     = reshape(thetagrid,1,ntot);
shootgridflat = shoot.mapaccum('map',ntot);

out = shootgridflat(5,vflat,thetaflat/180*pi);
shootgrid = reshape(out,ntheta,nv);

figure;
contourf(vgrid,thetagrid,full(shootgrid))
colorbar()
title('Shooting distance [m]');
xlabel('initial speed [m/s]');
ylabel('Angle [deg]');

%% Task 4 =================================================================
% what is the best angle to shoot far
% let's start with a slice throught v = 35 [m/s]

ts = linspace(30,60,100);

shootsweep = shoot.mapaccum('map',100);
shootsweep.printDimensions();
out = shootsweep(7,35,ts/180*pi); % T = 7,v0 = 35,theta = sweep}
dist = full(out);

% shooting distance in function of theta. thetaOpt is not 45 due to 
% aerodynamic drag
figure;hold on;grid on;
plot(dist,ts);
plot(dist,45*ones(100,1))
xlabel('shooting distance [m]')
ylabel('shooting angle [deg]')

% Let's find the optimal angle as a function of initial speed
% shoot: (guess for T, initial speed, shooting angle) ->
%        impact time, shooting distance)
%              d shoot.distance
% optimal <-> ------------------ = 0 
%              d angle 

% Jshoot = shoot.jacobian(2,1); % compute derivative respect angle
% J_out = Jshoot(5,v,theta);  
% % again root finding problem
% residual2 = Function('residual2',{theta,v},{J_out});
% % implicit function with Newton Solver                  
% opt = rootfinder('impl','newton',residual2); 
% 
% optsweep = opt.mapaccum('map',10);
vs = linspace(0.1,50,10);
% out = optsweep(0.7,vs);
% ts = full(out)*180/pi;
% 
% figure;hold on;grid on;
% contourf(vgrid,thetagrid,full(shootgrid))
% plot(vs,45*ones(10))
% plot(vs,ts)
% colorbar()
% title('Shooting distance [m]');
% xlabel('initial speed [m/s]');
% ylabel('Angle [deg]');

%% Task 5
% Maximize my chance for a hole in one
% The hole is at 80m distance
% NLP program
% min f(x)
%    s.t.    lbx <=  x   <= ubx 
%            lbg <= g(x) <= ubg

V       = [v;theta;T];
x0      = [0;0;v*cos(theta);v*sin(theta)];
fly_out = fly('x0',x0,'p',T);
xf      = fly_out.xf;

f = 0; % initialization objective function
g = [xf(1:2);xf(4)]; % xf(1) = xf ,xf(2) = yf, xf(4) = vy 

%nlp = Function('nlp',{x} nlpIn('x',V),nlpOut('f',f,'g',g));
nlp = struct('f', f, 'x', V, 'g', g);
solver = nlpsol('solver','ipopt',nlp); % setting NLP program
% bounds 
%   80 <= px(T) <= 80
%    0 <= py(T) <= 0
% -inf <= vx(T) <= 0
x0  = [35;0.4;5];     % initial guess
lbg = [80;0;-inf];
ubg = [80;0;0];
lbx = [10;0;0];
ubx = [80;pi/2;inf];

disp('smalltext')
sol = solver('x0',x0, 'lbx',lbx,'ubx',ubx,'lbg',lbg,'ubg',ubg);
nlp_out = full(sol.x); % retrieve solution
nlp.f
nlp.g

fprintf('Opt: %0.3f [m/s] -- %0.2f deg', full(sol.x(1)),full(sol.x(2))*180/pi);

%% Minimization of uncertanty - using covariance matrix
% let's consider Gaussian perturbations on the initial values
sigma_v = 1;        % [m/s]
sigma_theta = 0.02; % [rad]

P0 = diag([sigma_v^2,sigma_theta^2]);
Jt = jacobian(xf(1:2),V);
J  = Jt(1,1:2)-Jt(1,3)/Jt(2,3)*Jt(2,1:2);

Pend = J*P0*J';

f = Pend
nlp = struct('f', f, 'x', V, 'g', g);
solver = nlpsol('solver','ipopt',nlp); % setting NLP program
disp('smalltext')

sol = solver('x0',x0, 'lbx',lbx,'ubx',ubx,'lbg',lbg,'ubg',ubg)
sols = full(sol.x); % retrieve solution

fprintf('Opt: %0.3f [m/s] -- %0.2f deg', sols(1),sols(2)*180/pi);
fprintf('sigma %0.3f m \n', sqrt(full(sol.f)));

% hold on;
% contourf(vgrid,thetagrid,full(shootgrid))
% plot(vs,45*ones(10));
% plot(vs,ts)
% plot(sols(1),sols(2),'or')
% colorbar()
% title('shooting distance [m]');
% xlabel('initial speed [m/s]');
% ylabel('angle [deg]');
