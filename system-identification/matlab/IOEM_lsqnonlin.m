function InOutError
% Model with Input and Output Errors
%% step 0: generate data from true system 
%clc;close all;
T = 1; ts = 0.1;
[t,u,y] = GenerateData(T,ts);
figure(1);
subplot(2,1,1);hold on;stairs(t,u);grid on;ylabel('u(t)');
subplot(2,1,2);hold on;plot(t,y);grid on;ylabel('y(t)');xlabel('t');


p = [-0.5;0.7;0.2;0.5];
u_guess = u;

[yguess] = simu(t,u,p); 

subplot(2,1,1);stairs(t,u,'.');grid on;
subplot(2,1,2);plot(t,yguess,'.');grid on;

theta0 = [p;u];

[res] = misfit(t,u,y,theta0);

[theta,~,resnorm] = lsqnonlin(@(theta) misfit(t,u,y,theta),theta0);

p = theta(1:4)

end

function [t,u,y] = GenerateData(T,ts)
b1 = 0.5; b0 = 0.2; a2 = 0.7; a1 = -0.5; a0 = 1;
sys = tf([b1 b0],[a2 a1 a0]);
% define input
t = [0:ts:T-ts]';
u0 = sin(2*pi*0.5*t)+cos(2*pi*2*t)+cos(2*pi*t);
sigma_u = 0.1;
u = u0 + sigma_u*randn(size(t));                  % add noise u = u0+nu
% generate output
y0 = lsim(sys,u,t);                               % generate y
sigma_y = 0.1;
y = y0 + sigma_y*randn(size(t));                  % add noise y = y0+ny
end

function [y] = simu(t,u,p)
% deterministic LTI system simulation for times t 
% with inputs u and parameters p (2nd order continuous time system)
a0 = 1;                        % define parameter
a1 = p(1);
a2 = p(2);
b0 = p(3);
b1 = p(4);
sys=tf([b1 b0],[a2 a1 a0]);  % guess system
y = lsim(sys,u,t); 
end

function [ res ] = misfit(t,u,y,theta)
% theta has to containt in this case the parameter vector as well as the 
% true control u_tilde
N = length(t);
sigma_u = 0.1;          
sigma_y = 0.1;
p=theta(1:4);                      % parameter b1 b0 a2 a1 a0
utilde = theta(5:4+N);             % true input
ysim = simu(t,utilde,p);

figure(1);
subplot(2,1,1);
stairs(t,u,'b'); hold on;
stairs(t,utilde,'r'); hold off;

subplot(2,1,2);
plot(t,y,'b'); hold on;
plot(t,ysim,'r'); hold off;

res = zeros(2*N,1);
res(1:N) = (utilde-u)/sigma_u;      % normalize input
res(N+1:2*N) = (ysim-y)/sigma_y;    % notmalize output

end


