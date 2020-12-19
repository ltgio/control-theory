%% Simple Linear Least Square
clear all;close all;clc; 
N = 30;             
x = linspace(0,10,N)';

% true parameters
m = 1;
b_est = 0.5;
% generate data
y = m.*x+b_est + randn(N,1);

%% apply LLS
Phi = [x,ones(N,1)];
     
theta_est = inv(transpose(Phi)*Phi)*transpose(Phi)*y

m_est = theta_est(1);
b_est = theta_est(2);

y_fit = m_est.*x + b_est

plot(x,y,'ro');
hold on;grid on;
plot(x,y_fit);
legend('y measurement','fit model')
