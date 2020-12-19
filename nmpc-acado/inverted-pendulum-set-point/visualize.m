function [] = visualize(time, state_sim, Xref, xmin, xmax)

l = 0.8;

figure(10); clf
whitebg([1.0 1.0 1.0])
set(gcf,'Color',[1 1 1])

x_r = Xref(1,1);
y_r = 0;

line([xmin xmax], [0 0], 'color', 'k', 'Linewidth',1.5); hold on;
line([xmin xmin], [-0.1 0.1], 'color', 'k', 'Linewidth',1.5); hold on;
line([xmax xmax], [-0.1 0.1], 'color', 'k', 'Linewidth',1.5); hold on;

plot(x_r, y_r, 'gx', 'MarkerSize', 16, 'Linewidth', 2);
text(1.5-0.08,0.8+0.045,['current time: ' num2str(time(end)) 's'],'FontSize',15);
plot(state_sim(end,1),0,'ks','MarkerSize',30,'Linewidth',3);

theta = state_sim(end,2);
xB = state_sim(end,1)-l*sin(theta);
yB = l*cos(theta);

line([state_sim(end,1) xB], [0 yB], 'Linewidth',2);
plot(xB,yB,'ro','MarkerSize',25,'Linewidth',3);

grid on;
xlim([xmin-0.5 xmax+0.5]);
ylim([xmin-0.5 xmax+0.5]);