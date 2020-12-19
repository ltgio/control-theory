%% Plot Eigenvalues two chaotic system
clc;clear all;close all;
global x0
x0 = [0.02;0.05;0.04;0.0;1.0;0.0];  % initial state
%% Nonlinear simulation using the RK4 integrator
T = 20;
N = 2000;
Ts = T/N;
input.Ts = Ts;
input.nSteps = 2;

%% Discretize and linearize the dynamic system:
x_lin = zeros(6,1); u_lin = zeros(3,1);
input.x = x_lin; input.u = u_lin;
output = RK4_integrator( @ode, input );

Ad = output.sensX;
Bd = output.sensU;

%% Design of the LQR controller using dlqr:
C = [1 0 0 -1  0  0;
     0 1 0  0 -1  0;
     0 0 1  0  0 -1]; 
 
Q = 100*C'*C;R = 0.01*eye(3);beta = 0.9;
[K,P,E] = dlqr(sqrt(beta)*Ad,sqrt(beta)*Bd,Q,R);

EigOpen = eig(Ad);
Re = real(EigOpen);
Im = imag(EigOpen);

figure(1);grid on;hold on;
% build circle
ang=0:0.01:2*pi; 
xp=1*cos(ang);
yp=1*sin(ang);
plot(0+xp,0+yp,'--');
% plot eigenvalue
plot(Re([1:3]),Im([1:3]),'rx');
plot(Re([4:6]),Im([4:6]),'go');
legend ('unit circle','eigenvalue master system','eigenvalue slave system')
xlabel('Real');ylabel('Imag');title('{\bf Plot Eigenvalues}')
axis([-1.5 1.5 -1.5 1.5]);

EigClose = eig(sqrt(beta)*Ad-sqrt(beta)*Bd*K);
Re = real(EigClose);
Im = imag(EigClose);

figure(2);grid on;hold on;
% build circle
ang=0:0.01:2*pi; 
xp=1*cos(ang);
yp=1*sin(ang);
plot(0+xp,0+yp,'--');
% plot eigenvalue
plot(Re([1:3]),Im([1:3]),'rx');
plot(Re([4:6]),Im([4:6]),'go');
legend ('unit circle','eigenvalue master system','eigenvalue slave system')
xlabel('Real');ylabel('Imag');title('{\bf Plot Eigenvalues}')
axis([-1.5 1.5 -1.5 1.5]);




