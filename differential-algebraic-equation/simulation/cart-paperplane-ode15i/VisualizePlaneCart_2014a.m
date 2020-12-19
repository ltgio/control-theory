function [ output_args ] = VisualizePlaneCart_2014a(tsim,Xsim)

    xPlane       = Xsim(:,1);         
    zPlane       = Xsim(:,2);          
    xCart        = Xsim(:,3); 
    vxPlane      = Xsim(:,4);
    vzPlane      = Xsim(:,5);
    vxCart       = Xsim(:,6);
    T            = Xsim(:,7);
    
    velocity     = [vxPlane vzPlane vxCart]; 
    
    zCart        = 5*ones(length(Xsim),1);
    vzCart       = zeros(length(Xsim),1);

%% Plot 2D trajecotory
    figure;hold on;grid on;                           
    xlim([0 200]);
    ylim([-100 100]);
    xlabel('x [m]');
    ylabel('h [m]');
    ax = gca;
    % Initialize Plot 
    trajPlane = plot(xPlane,-zPlane,'Color','b');    % plot Plane Trajectory
    trajCart  = plot(xCart, zCart, 'Color','r');     % plot Cart Trajectory 
    Plane     = plot(xPlane(1),-zPlane(1), '>'     ,'MarkerFaceColor','b','MarkerSize',15,'Parent',ax);
    Cart      = plot(xCart(1),  zCart(1) , 'square','MarkerFaceColor','r','MarkerSize',15,'Parent',ax);
    cable     = line([xCart(1),xPlane(1)],[5,-zPlane(1)],'LineWidth',1,'Color','k','Parent',ax);
    legend('Plane Trajectory','Cart Trajectory','Plane','Cart','cable');
        
    for k = 2:length(tsim)  
        tt = tic;
        set(trajPlane, 'XData', xPlane(1:k));
        set(trajPlane, 'YData', -zPlane(1:k));
        set(trajCart, 'XData',  xCart(1:k));
        set(trajCart, 'YData',  zCart(1:k));
        set(Plane, 'XData',     xPlane(k));
        set(Plane, 'YData',     -zPlane(k));        
        set(Cart, 'XData',      xCart(k));
        set(Cart, 'YData',      zCart(k));       
        set(cable, 'XData',     [xCart(k), xPlane(k)]);
        set(cable, 'YData',     [zCart(k),-zPlane(k)]);        
        pause(tsim(k)-tsim(k-1)-toc(tt))
        drawnow
    end
    %waitforbuttonpress
    figure;title('tether length in time');
    tetherLenght = sqrt((xPlane - xCart).^2+(zPlane - 0 ).^2);
    plot(tsim,tetherLenght);xlabel('time [s]');ylabel('length [m]');grid on;
end

