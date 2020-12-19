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