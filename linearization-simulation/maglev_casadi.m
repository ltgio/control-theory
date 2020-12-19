function MagLev_casadi
% ----------------------------------------------------------------------- %
%     [COMPLETE] Trim,Linearization and Forward Simulation  via casadi    %
%                   Model: Magnetic Levitation System                     %
% ----------------------------------------------------------------------- %

clc;close all;

%% Create System
[y,dx,Fode,x,u,nx,nu] = MagLev;

%% Find Equilibrium Point [ball 1 meter above the reference]
position = 1;

w    = [x;u]; 
w0   = [position;   0;   0;   0];
wmin = [position;-inf;-inf;-inf];
wmax = [position; inf; inf; inf];

[xeq,ueq] = trim(dx,w,w0,wmin,wmax,nx);

%% Linearize system
[sysLTI,FsysLTI] = linearize(dx,y,x,u,xeq,ueq);
sysLTI

%% Create RK4 integrator
ts = 0.01;
M  = 4;
N  = 100;
time = linspace(0,N*ts,N);
[RK4_Step1_NL,RK4_StepN_NL] = getIntegrator(Fode,x,u,ts,M,N);
[RK4_Step1_L ,RK4_StepN_L]  = getIntegrator(FsysLTI,x,u,ts,M,N);

%% Simulate NL-linear and nonLinear Model
x0 = zeros(nx,1);
A  = 2;
uN = [-A.*ones(N/4,nu);A.*ones(N/4,nu);zeros(N/2,nu)];
figure;plot(time,uN,'LineWidth',2);grid on;

% N steps one by one
XsimNL = zeros(nx,N);
XsimL  = zeros(nx,N);

XsimNL(:,1) = x0 + xeq;
XsimL(:,1)  = x0;

for i = 1:N-1
  XsimNL(:,i+1) = full(RK4_Step1_NL(XsimNL(:,i), uN(i) + ueq));
  XsimL(:,i+1)  = full(RK4_Step1_L(XsimL(:,i)  , uN(i)      ));
end

figure;hold on;grid on;
plot(time,XsimNL      ,'LineWidth',2)
plot(time,XsimL + xeq ,'LineWidth',2,'LineStyle','-.')

% N steps in one shoot: input uN and x0
XsimNL = full(RK4_StepN_NL(x0 + xeq, uN + ueq));
XsimL  = full(RK4_StepN_L( x0,       uN      ));

figure;hold on;grid on;
plot(time,XsimNL      ,'LineWidth',2)
plot(time,XsimL + xeq ,'LineWidth',2,'LineStyle','-.')

end

function [y,dx,Fode,x,u,nx,nu] = MagLev
import casadi.*
%% Define Model
g0 = 9.81;  R = 50;    L = 0.5;
m  = 0.02; Km = 19.62; b = 0.1;

% ueq = 5;
% xeq = [sqrt(Km/(m*g0))*(ueq/R);0;ueq/R];

nx = 3;
nu = 1;

x  = MX.sym('x',nx);
u  = MX.sym('u',nu); % voltage  [V]

x1 = x(1); % position [m]
x2 = x(2); % velocity [m/s]
x3 = x(3); % current  [A]

dx = [x2;
      g0-(b/m)*x2-(Km/m)*((x3/x1)^2);
     (1/(L+Km/x1))*(-R*x3+Km*(x2/(x1^2))+u)];  

Fode = Function('ode',{x,u},{dx});
y    = x1;  % measure position
end
function [xeq,ueq] = trim(dx,w,w0,wmin,wmax,nx)
  import casadi.*
  Fw     = dx'*dx;         
  nlp    = struct('x',w, 'f',Fw);
  solver = nlpsol('solver','ipopt', nlp);
  sol    = solver('x0', w0, 'lbx', wmin, 'ubx', wmax); % trim

  if(full(sol.f)<eps)
      disp('--------------------------------------------------------------');
      disp('Equilibrium Point Found');
      w_opt = full(sol.x);     
      xeq   = round(w_opt(1:nx),3);
      ueq   = round(w_opt(nx+1:end),3);
      disp(['xeq = ',num2str(xeq')]);
      disp(['ueq = ',num2str(ueq')]);
  else
      assert('Infeable problem')    
  end
end
function [sysLTI,FsysLTI] = linearize(dx,y,x,u,xeq,ueq)
import casadi.*
A_fun = Function('A',{x,u},{jacobian(dx,x)},char('x','u'), char('A'));
A     = full(A_fun(xeq,ueq));

B_fun = Function('B',{x,u},{jacobian(dx,u)},char('x','u'), char('B'));
B     = full(B_fun(xeq,ueq));

C_fun = Function('C',{x,u},{jacobian(y,x)} ,char('x','u'), char('C'));
C     = full(C_fun(xeq,ueq));

D_fun = Function('D',{x,u},{jacobian(y,u)} ,char('x','u'), char('D'));
D     = full(D_fun(xeq,ueq));

sysLTI = ss(A,B,C,D,...
         'StateName',{'p','v','i'},...
         'StateUnit',{'[m]','[m/s]','[A]'},...
         'InputName',{'V'},...
         'InputUnit',{'Volt'});

FsysLTI = Function('FsysLTI',{x,u},{A*x+B*u},char('x','u'),char('dx'));
end
function [RK4_Step1,RK4_StepN] = getIntegrator(Fode,x,u,ts,M,N)
    import casadi.*
    dt = ts/M; 
    k1 = Fode(x,u);
    k2 = Fode(x+dt/2.0*k1,u);
    k3 = Fode(x+dt/2.0*k2,u);
    k4 = Fode(x+dt*k3    ,u);
    xf = x + dt/6.0*(k1+2*k2+2*k3+k4);
    % Create a function that simulates one step propagation in a sample
    RK4 = Function('RK4',{x,u},{xf});
    X = x;
    for i=1:M
        X = RK4(X,u);
    end
    % Create a function that simulates all step propagation on a sample
    Integrator = Function('sample1',{x,u},{X},char('x(k)','u(k)'),char('x(k+1)'));
    % speedup trick: expand into scalar operations
    RK4_Step1 = Integrator.expand();
    % choose number of simulation step % Optional
    RK4_StepN = RK4_Step1.mapaccum('samplesN', N); 
    
    disp(['Runge-Kutta Integrator order M = ',num2str(M),...
          ' with sample time ts = ',num2str(ts),'[s], sample N = ',N,'created...'])
end