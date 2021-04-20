function OutputErrorMethod
clc;close all;
%==========================================================================
% Author: Marie Curie PhD student Giovanni Licitra
% Title: Output Error Method (using Optimization Toolbox)
% Data: 18/01/2016
% Description: Estimation of dynamic system xdot = a*x + b*u with nx=1,nu=1
% with x(t)=x0+xn
%==========================================================================
global iter
iter = 0;

a = -0.5;b = 1;nx = 1; 
p = [a;b];                                       % true parameters
T = 5;ts = 0.01;t = [0:ts:T-ts]';                % final time | sample time | array time
u = sin(2*pi*t)+cos(2*pi*3*t);ut = t;            % input time varying
x0 = 0.1;                                        % initial condition
[t,x] = ode45(@(t,x) ode(t,x,u,ut,p,1),t, x0);   % compute forward simulation
noise = 0.2*randn(size(t));                      % noise
x = x + noise;                                   % add noise to the state

[t_eul,x_eul] = Euler(T,ts,nx,x0,p,u,ut);        % Euler integrator
%[res] = fitting(T,ts,nx,x0,p,u,ut,x);           % |y-M(theta)|

theta0 = [0;0];                                  % initial guess
lb = [-1;0];                                     % lower bound for theta
ub = [0;1];                                      % upper bound for theta

options = optimoptions(@fmincon,'MaxIter',30);
[theta,resnorm,residual,~,output,~] = lsqnonlin(@(theta) ...
    fitting(T,ts,nx,x0,theta,u,ut,x),theta0,lb,ub,options);

[t_est, x_est] = Euler(T,ts,nx,x0,theta,u,ut);

figure(2);
disp(sprintf('theta = %0.2f',theta));
disp(sprintf('standard deviation residual = %0.2f',std(residual)))
subplot(2,1,1);
hold on;plot(t,x,'.');plot(t_eul,x_eul);plot(t_est,x_est);
legend('x_{data}','x_{Euler}','x_{fitting}');
grid on;xlabel('time [s]');ylabel('x');
subplot(2,1,2);
plot(t,residual);xlabel('time [s]');ylabel('residual');grid on;
legend(sprintf('resnorm = %0.2f',resnorm));
end

function [t_eul, x_eul] = Euler(T,h_eul,nx,x0,p,u,ut)   % Euler integrator
    t_eul = [0:h_eul:T-h_eul]';                         % time Euler
    x_eul = zeros(length(t_eul),nx);                                         

    x_eul(1,:) = [x0];                                  % initial condition
    for i=1:length(t_eul)-1
        dx = ode(0,x_eul(i,:),u(i,:),ut(i,:),p,0);
        x_eul(i+1,:) = x_eul(i,:)' + dx * h_eul;
    end
end
function dx = ode(t,x,u,ut,p,opt)                       % simple ODE
    % input argument
    % t = input | x = state | u = input | ut = u(t)
    % opt = 1 for forward simulation by ode45 with input time varying
    % opt = 0 for other integration home made
    
    if opt==1
        u = interp1(ut,u,t);         % Interpolate the data set (ut,u) at time t
    end
    a = p(1); b = p(2);
    dx = a*x+b*u;
end
function [res] = fitting(T,ts,nx,x0,theta,u,ut,xdata)   % |y-M(theta)|
% This function is called by lsqnonlin.
% x is a vector which contains the coefficients of the
% equation.  
global iter

[t_est,x_est] = Euler(T,ts,nx,x0,theta,u,ut);   % Euler integrator
iter = iter + 1;

fig = figure(1);
plot(t_est,xdata,'.');hold on;plot(t_est,x_est);title(sprintf('# of Iteration = %0.2f',iter));hold off;
legend('x_{data}','x_{fitting}');
grid on;xlabel('time [s]');ylabel('x');

%saveas(fig,sprintf('fig%0.3d.jpg',iter))  % here you save the figure

res = (x_est - xdata);
pause(0.2);

end

