function updatePlot(xsim,usim,time,plot_p,plot_v,plot_i,plot_u,plot_Ball)

    position  = round(xsim(:,1),3);
    velocity  = round(xsim(:,2),3);
    current   = round(xsim(:,3),3);
    voltage   = round(usim(:,1),3);
    
    set(plot_p   , 'XData',  time);
    set(plot_p   , 'YData',  position);
    set(plot_v   , 'XData',  time);
    set(plot_v   , 'YData',  velocity);
    set(plot_i   , 'XData',  time);
    set(plot_i   , 'YData',  current);
    set(plot_u   , 'XData',  time);
    set(plot_u   , 'YData',  voltage);
    set(plot_Ball, 'XData',  0);
    set(plot_Ball, 'YData',  position(end));
    drawnow
end