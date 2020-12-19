%% TITLE: OUTPUT ERROR METHOD
% Model: xdot = a*x + b* u
%        with parameters = [a;b], 
%        x0 = initial condition
%        theta = [a;b;x0] unknown parameters
%
% This example uses more advanced constructs than the vdp* examples:
% Since the number of control intervals is potentially very large here,
% we use memory-efficient Map and MapAccum, in combination with
% codegeneration.
%
% We will be working with a 2-norm objective:
% || y_measured - y_simulated ||_2^2
%
% This form is well-suited for the Gauss-Newton Hessian approximation.

% Version:     Matlab 2014b/optistack-casadi3.0.0-rc3
% Author:      Marie Curie PhD student Giovanni Licitra
% Data:        03-03-2016

clear all;close all;clc;
import casadi.*
%% SETTINGS ===============================================================
N  = 1000;                          % Number of samples
fs = 10;                           % Sampling frequency [hz]
t  = linspace(0,(N-1)*(1/fs),N)';   % time array

N_steps_per_sample = 10;   
dt = 1/fs/N_steps_per_sample;       % integration step for ode

nx = 1;                             % n states
nu = 1;                             % n inputs

rng(1)   
%% Model parameters =======================================================
a      = -0.5;
b      = 1;
x_init = 0.1;

param_truth = [a;b];             % True parameters
theta_truth = [a;b;x_init];
theta_guess = [-5;5;1];          % guess for NLP
%scale = [1e-6;1e-4;1];          % scaling factor NOT REQUIRED

%% Model generation via casADi/OPTistack ==================================
x  = MX.sym('x');
u  = MX.sym('u');

states   = x;
controls = u;

% define uknow parametes as design variable
a  = optivar(); 
b  = optivar();
x0 = optivar();
param  = [a;b];                
theta  = [a;b;x0]; % store uknow parameters

% xdot = f(x,u,p) <==> rhs = f(x,u,p)
rhs = a*x+b*u; 
% Form an ode function
ode = Function('ode',{states,controls,param},{rhs});

%% build integrator: RK4 ==================================================
k1 = ode(states          ,controls,param);
k2 = ode(states+dt/2.0*k1,controls,param);
k3 = ode(states+dt/2.0*k2,controls,param);
k4 = ode(states+dt*k3    ,controls,param);
xf = states + dt/6.0*(k1+2*k2+2*k3+k4);
% Create a function that simulates one step propagation in a sample
one_step = Function('one_step',{states, controls, param},{xf});

X = states;
for i=1:N_steps_per_sample
    X = one_step(X, controls, param);
end

% Create a function that simulates all step propagation on a sample
one_sample = Function('one_sample',{states, controls, param}, {X});
% speedup trick: expand into scalar operations
one_sample = one_sample.expand();

%% Compute Forward Simulation =============================================
% choose number of simulation step
all_samples = one_sample.mapaccum('all_samples', N);
u_data      = sin(2*pi*t)+cos(2*pi*3*t)+0.5*rand(N,1); % signal excitation: sum of two sine+noise
% u_data = 0.1*rand(N,1);                         % signal excitation: random noise
x_init      = DM([x_init]);                       % Initial Condition x0 = [0.1]; [nx = 1]

% perform forward simulation
X_measured =  all_samples(x_init, u_data, repmat(param_truth,1,N));             
 
% OPTION 1: gaussian noise
nx = 0.1*randn(N,1);                      
% OPTION 2: coloured noise: frequency information required 
%nx = 0.1*filter(1,[1 -0.9],randn(N,1));  
 
% sum noise measurements to the state
y_data = X_measured' + nx; 
 
%% Plot Forward Simulation ================================================
figure;
subplot(2,1,1);title('Forward Simulation: True Parameters');hold on;
               plot(t,optival(X_measured)','b');
               plot(t,optival(y_data)','g.');
               grid on;legend('x','y = x_{1} + x_{n}');
               ylabel('')
subplot(2,1,2);stairs(t,optival(u_data)','r');grid on;legend('u');          
               xlabel('time [s]');
                
%% Set Identification Algorithm =========================================== 
% Single Shooting Strategy
X_symbolic = all_samples(x0, u_data, repmat(param,1,N));      
e_time     = y_data - X_symbolic';  
% provide initial guess for the data fitting
a.setInit(theta_guess(1));
b.setInit(theta_guess(2));
x0.setInit(theta_guess(3));

% set option 
options = struct;
options.codegen = false;
% Hand in a vector objective -> interpreted as 2-norm
% such that Gauss-Newton can be performed
% PERFORM DATA FITTING
optisolve(e_time,{},options);      
% retrieve estimated parameters
a_est     = optival(a);
b_est     = optival(b);
x0_est    = optival(x0);
param_est = [a_est;b_est];
 
%% check estimation performance ===========================================
residual      = optival(e_time);
% Compute mean, variance and Quadratic Mean
mean_time     = mean(residual);      
variance_time = var(residual); 
RMS           = rms(residual); 

% Retrieve Forward Simulation with estimated Parameters ===================
X_est = full(all_samples(x0_est, u_data, repmat(param_est,1,N)))';      

% Compute Power Spectral Density
xdft_res          = fft(residual);
xdft_res          = xdft_res(1:N/2+1,:);
psdx_res          = (1/(fs*N))*abs(xdft_res).^2;
psdx_res(2:end-1) = 2*psdx_res(2:end-1);
freqPlot          = 0:fs/N:fs/2;

%% Plot residuals [only time information] =================================
figure;
subplot(2,2,1);title(['Residual: RMS = ',num2str(RMS),' | mean = ',num2str(mean_time),' | variance = ',num2str(variance_time)]);
               hold on;plot(t,residual,'y');grid on;xlabel('time [s]');
subplot(2,2,[2 4]);title('Residual (time info): Periodogram Frequency Domain');
               hold on;plot(freqPlot,10*log10(psdx_res),'c');grid on;
               xlabel('rad/s');ylabel('Power/Frequency [dB/Hz]');
subplot(2,2,3);title('y_measurement vs y_simulated');hold on;
               plot(t,optival(y_data)','c.');grid on;
               plot(t,X_est,'b');legend('y_{m}','y_{sim}');
               
% Print some information ==================================================
disp('');
disp('True parameters')
str = '%s* = %f \n';Cdisp = {'a','b','x0';theta_truth(1),theta_truth(2),theta_truth(3)};
disp(sprintf(str,Cdisp{:}));

disp('Estimated parameters')
str = '%s* = %f \n';Cdisp = {'a','b','x0';a_est,b_est,x0_est};
disp(sprintf(str,Cdisp{:}));


