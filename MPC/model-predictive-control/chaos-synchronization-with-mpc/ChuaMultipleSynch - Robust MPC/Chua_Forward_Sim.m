function Chua_Forward_Sim
%% Title: forward simulation different chaotic system
%  master:
%  slave: Lorenz system
clc;clear all;close all;
x0 = [0.9365;-0.061;0.1889;0.02;0.05;0.04];  % initial state
u = [0;0;0];
T = 20;
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
u = zeros(3,N+1);
visualizeChua(t_rk4,x_rk4,u)
end

function dx = ode(t,x,u)      
    % uncertainty to slave system
    alpha = 15.6;beta = 28;m0 = -1.143;m1 = -0.714; % Master: Chua's circuit Parameters                                        
    p = 10;q = 100/7;                               % slave system parameter
    
    x1 = x(1);y1 = x(2);z1 = x(3);
    
    x2 = x(4);y2 = x(5);z2 = x(6);
    ux = u(1);uy = u(2); uz = u(3);
  
    f_x1 = m1*x1+0.5*(m0-m1)*(abs(x1+1)-abs(x1-1)); % non linearita master
    f_x2 = 2*x2^3 - x2/7;                           % non linearita slave
    
    dx = [alpha*(y1-x1-f_x1);% 
          x1 - y1+ z1;
          -beta*y1;
          p*(y2-f_x2)+ux;        
          x2-y2+z2+uy;
          -q*y2+uz];  
end

function [] = visualizeChua(t_rk4,x_rk4,u)

e1 = x_rk4(1,:) - x_rk4(4,:); 
e2 = x_rk4(2,:) - x_rk4(5,:);
e3 = x_rk4(3,:) - x_rk4(6,:);

figure(1);
subplot(3,3,1);plot(t_rk4,x_rk4(1,:),'LineWidth',2);hold on;
               plot(t_rk4,x_rk4(4,:),'LineWidth',2);grid on;
               legend('x1','x2');

subplot(3,3,2);plot(t_rk4,x_rk4(2,:),'LineWidth',2);hold on;
               plot(t_rk4,x_rk4(5,:),'LineWidth',2);grid on;
               legend('y1','y2');
               
subplot(3,3,3);plot(t_rk4,x_rk4(3,:),'LineWidth',2);hold on;
               plot(t_rk4,x_rk4(6,:),'LineWidth',2);grid on;
               legend('z1','z2');
               
subplot(3,3,4);plot(t_rk4,e1,'g','LineWidth',2);hold on;
               legend('e1');grid on;
subplot(3,3,5);plot(t_rk4,e2,'g','LineWidth',2);hold on;
               legend('e2');grid on;
subplot(3,3,6);plot(t_rk4,e3,'g','LineWidth',2);hold on;
               legend('e3');grid on;
               
subplot(3,3,7);plot(t_rk4(1:end),u(1,:),'m','LineWidth',2);hold on;
               legend('ux');grid on;
subplot(3,3,8);plot(t_rk4(1:end),u(2,:),'m','LineWidth',2);hold on;
               legend('uy');grid on;
subplot(3,3,9);plot(t_rk4(1:end),u(3,:),'m','LineWidth',2);hold on;
               legend('uz');grid on;

figure(2);
plot3(x_rk4(1,:),x_rk4(2,:),x_rk4(3,:),'LineWidth',2);hold on;
plot3(x_rk4(4,:),x_rk4(5,:),x_rk4(6,:),'LineWidth',2);grid on;
xlabel('x(t)');ylabel('y(t)');zlabel('z(t)');
legend('Master System','Slave System');

figure(3);
plot(x_rk4(1,:),x_rk4(2,:),'LineWidth',2);hold on;
plot(x_rk4(4,:),x_rk4(5,:),'LineWidth',2);grid on;
legend('Master System','Slave System');

end