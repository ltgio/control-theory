function [ output_args ] = VisualizeMultiNode(tsim,Xsim)


    % get number of nodes
    N = (size(Xsim, 2) - 6) / 4 + 1;

    xCart       = Xsim(:,1);         
    vxCart       = Xsim(:,2);      
    
    xNodes = [];
    zNodes = [];
    vxNodes = [];
    vzNodes = [];
    index = 2;
    for k=1:N
      xNodes(:,k)  = Xsim(:,index+1);
      zNodes(:,k)  = Xsim(:,index+2);
      vxNodes(:,k) = Xsim(:,index+3);
      vzNodes(:,k) = Xsim(:,index+4);
      index = index + 4;
    end
    xPlane       = Xsim(:,index-3);
    zPlane       = Xsim(:,index-2);
    vxPlane      = Xsim(:,index-1);
    vzPlane      = Xsim(:,index-0);
    zCart        = 0*ones(size(Xsim, 1),1);

%% Plot 2D trajecotory
    figure(1);hold on;grid on;                           
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
    
    
    cart_cable     = line([xCart(1),xNodes(1,1)],[zCart(1),-zNodes(1,1)],'LineWidth',2,'Color','k','Parent',ax);
    cable_handles = [];
    for i=1:N-1
      cable     = line([xNodes(1,i),xNodes(1,i+1)],[-zNodes(1,i),-zNodes(1,i+1)],'LineWidth',2,'Color','k','Parent',ax);
      cable_handles = [cable_handles, cable];
    end
    
    plane_cable = line([xNodes(1,end),xPlane(1)],[zNodes(1,end),-zPlane(1)],'LineWidth',1,'Color','k','Parent',ax);
    legend('Plane Trajectory','Cart Trajectory','Plane','Cart','cable');
        
    for k = 2:length(tsim)  
        set(trajPlane, 'XData', xPlane(1:k));
        set(trajPlane, 'YData', -zPlane(1:k));
        set(trajCart, 'XData',  xCart(1:k));
        set(trajCart, 'YData',  zCart(1:k));
        set(Plane, 'XData',     xPlane(k));
        set(Plane, 'YData',     -zPlane(k));        
        set(Cart, 'XData',      xCart(k));
        set(Cart, 'YData',      zCart(k));       
    
        set(cart_cable, 'XData', [xCart(k), xNodes(k,1)]);
        set(cart_cable, 'YData', [zCart(k), -zNodes(k,1)]);
        for i=1:N-1
          set(cable_handles(i), 'XData', [xNodes(k,i), xNodes(k,i+1)]);
          set(cable_handles(i), 'YData', [-zNodes(k,i), -zNodes(k,i+1)]);
        end
        
        set(plane_cable, 'XData', [xNodes(k,end), xPlane(k)]);
        set(plane_cable, 'YData', [-zNodes(k,end), -zPlane(k)]);

        xlim([xCart(k)-50 xCart(k)+50]);
        ylim([-50 50]);
        pause(tsim(k)-tsim(k-1))
        drawnow
    end
end

