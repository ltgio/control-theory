function [plot_p,plot_v,plot_i,plot_u,plot_Ball] = initializePlot(x0,u0,time)
    figure('units','normalized','outerposition',[0 0 1 1])
    subplot(3,2,1);
    plot_p = plot(time,x0(1).*ones(length(time),1),'Color','b','LineWidth',1.5);ylabel('position [m]');
    xlabel('time [s]');xlim([0,time(end)]);grid on; 
    subplot(3,2,3);
    plot_v = plot(time,x0(2).*ones(length(time),1),'Color','b','LineWidth',1.5);ylabel('velocity [m/s]');
    xlabel('time [s]');xlim([0,time(end)]);grid on;
    subplot(3,2,5);
    plot_i = plot(time,x0(3).*ones(length(time),1),'Color','b','LineWidth',1.5);ylabel('current [A]');
    xlabel('time [s]');xlim([0,time(end)]);grid on;
    subplot(3,2,2);
    plot_u = plot(time,u0(1).*ones(length(time),1),'Color','r','LineWidth',1.5);ylabel('voltage [V]');
    xlabel('time [s]');xlim([0,time(end)]);grid on;

    x1_min =  0.9;  x1_max =  2.1; % plate
    subplot(3,2,[4,6]);hold on;grid on;
    line([-0.5,0.5],[x1_min,x1_min],'LineWidth',4,'Color','b');
    line([-0.5,0.5],[x1_max,x1_max],'LineWidth',4,'Color','b');
    plot_Ball = plot(0,x0(1),'Marker','o','MarkerEdgeColor','k','MarkerFaceColor','y','MarkerSize',10);
    axis([-1, 1, 0, 3]);ylabel('y [m]');
    
    set(plot_p   , 'XData',  0);
    set(plot_p   , 'YData',  x0(1));
    set(plot_v   , 'XData',  0);
    set(plot_v   , 'YData',  x0(2));
    set(plot_i   , 'XData',  0);
    set(plot_i   , 'YData',  x0(3));
    set(plot_u   , 'XData',  0);
    set(plot_u   , 'YData',  u0(1));
    set(plot_Ball, 'XData',  0);
    set(plot_Ball, 'YData',  x0(1));
    drawnow
    
    pause(1)
    
    
end