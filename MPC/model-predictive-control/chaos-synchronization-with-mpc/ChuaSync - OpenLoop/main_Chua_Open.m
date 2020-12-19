% main_ChaosSynch
clc;clear all;close all;
%x0 = [0.02;0.05;-0.04;0.6;-0.5;-0.0004];  % initial state
x0 = [0.02;0.05;0.04;0.0002;0.0005;0.0004];  % initial state

u = [0;0;0];
T = 80;

%% Nonlinear simulation using the RK4 integrator
N = 2000;
Ts = T/N;
x_rk4 = x0;
input.Ts = Ts;
input.nSteps = 2;
input.u = u;
t_rk4 = [0:N].*Ts;
for i = 1:N
    input.x = x_rk4(:,end);
    output = RK4_integrator( @ode, input );
    x_rk4(:,end+1) = output.value;
end

u = zeros(3,N);
visualizeChua(t_rk4,x_rk4,u)