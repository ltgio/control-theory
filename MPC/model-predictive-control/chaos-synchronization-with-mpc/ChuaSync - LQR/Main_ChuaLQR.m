%% main_ChaosSynch
clc;clear all;close all;
x0 = [0.2;0.1;-0.3;0.3;0.5;-0.2];
u = [0;0;0];
T = 5;

%% Nonlinear simulation using the RK4 integrator
N = 80;
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

figure;
plot(t_rk4',x_rk4','x','MarkerSize',10);

xlabel('time(s)')
ylabel('error')
legend('ode45', 'RK4');
grid on;

%% Discretize and linearize the dynamic system:
x_lin = zeros(6,1); u_lin = zeros(3,1);
input.x = x_lin; input.u = u_lin;
output = RK4_integrator( @ode, input );

A = output.sensX;
B = output.sensU;

%% Design of the LQR controller using dlqr:
C = [1 0 0 -1 0 0;
     0 1 0  0 -1 0;
     0 0 1  0  0 -1]; 
Q = 100*C'*C;

R = 1e-2*eye(3);

beta = 0.7;
[K,P] = dlqr(sqrt(beta)*A,sqrt(beta)*B,Q,R);

open('Chua_LQr.slx')