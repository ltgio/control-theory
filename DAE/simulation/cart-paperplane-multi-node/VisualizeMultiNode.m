function [ output_args ] = VisualizeMultiNode(tsim,Xsim,Tsim,Asim)   
    % get number of nodes
    N = (size(Xsim, 2) - 6) / 4 + 1;

    xCart        = Xsim(:,1);         
    vxCart       = Xsim(:,2);      
    
    xNodes       = [];
    zNodes       = [];
    vxNodes      = [];
    vzNodes      = [];
    index        = 2;
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
    
    %% Set Bound;
    alphaMin = -5;
    alphaMax = 15;
    pzMin    = 40*ones(length(zPlane),1);
%% Plot 2D trajecotory
    figure(1);
    %% Initialize Plot trajectory ------------------------------------------ 
    axTraj = subplot(6,2,[1,3,5,7,9,11]);hold on;grid on;                           
    xlabel('x [m]');ylabel('h [m]');
    ax = gca;
    trajPlane      = plot(xPlane(1),-zPlane(1),'Color','b','LineWidth',2);    % plot Plane Trajectory
    trajCart       = plot(xCart(1),  zCart(1), 'Color','r','LineWidth',2);     % plot Cart Trajectory 
    Plane          = plot(xPlane(1),-zPlane(1), '>'     ,'MarkerFaceColor','b','MarkerSize',15,'Parent',ax);
    Cart           = plot(xCart(1),  zCart(1) , 'square','MarkerFaceColor','r','MarkerSize',15,'Parent',ax);
    cart_cable     = line([xCart(1), xNodes(1,1)],[zCart(1),-zNodes(1,1)],'LineWidth',2,'Color','k','Parent',ax);
    PzTraj         = plot(xPlane(1),pzMin(),'b-.','LineWidth',1.3);
    cable_handles  = [];
    for i=1:N-1
      cable         = line([xNodes(1,i),xNodes(1,i+1)],[-zNodes(1,i),-zNodes(1,i+1)],'LineWidth',1,'Color','k','Parent',ax);
      cable_handles = [cable_handles, cable];
    end
    %plane_cable = line([xNodes(1,end),xPlane(1)],[zNodes(1,end),-zPlane(1)],'LineWidth',1,'Color','k','Parent',ax);
    legend('Plane Trajectory','Cart Trajectory','Plane','Cart','cable','p_{z}(t_{f}) constraint');
    %% Initialize Plot angle of attack -------------------------------------
    axAlpha       = subplot(6,2,[2 4]);hold on;grid on;                           
    trajAlpha     = plot(tsim(1),Asim(1),'Color','g','LineWidth',2);    % plot alpha Trajectory
    plot(tsim,alphaMax*ones(length(tsim),1),'g');                     % bound alpha Trajectory
    plot(tsim,alphaMin*ones(length(tsim),1),'g');                     % bound alpha Trajectory
    %Alpha         = plot(tsim(1),Asim(1),'Color','g','LineWidth',2);    % 
    xlabel('t [s]');ylabel('\alpha [degree]');
    legend('\alpha')
    xlim(axAlpha,[0 tsim(end)]);
    ylim(axAlpha,[-8 20]);
    %% Initialize Plot velocity Plane -------------------------------------
    axVelocity     = subplot(6,2,[6 8]);hold on;grid on;                           
    traVxPlane     = plot(tsim(1),vxPlane(1),'Color','b','LineWidth',2);    % plot alpha Trajectory
    traVzPlane     = plot(tsim(1),vzPlane(1),'Color','k','LineWidth',2);    % plot alpha Trajectory
    traVxCart      = plot(tsim(1),vxCart(1) ,'Color','r','LineWidth',2);    % plot alpha Trajectory
    xlabel('t [s]');ylabel('m/s');
    legend('v_{x Plane}','v_{z Plane}','v_{x Cart}');
    xlim(axVelocity,[0 tsim(end)]);
    ylim(axVelocity,[-5 20]);
    %% Initialize Plot Tension --------------------------------------------
    axTension       = subplot(6,2,[10,12]);hold on;grid on;                           
    trajTension     = plot(tsim(1),Tsim(1),'Color','m','LineWidth',2);    % plot alpha Trajectory
    xlabel('t [s]');ylabel('Tension [N]');
    legend('T [N]');
    xlim(axTension,[0 tsim(end)]);
    ylim(axTension,[-20 80]);
    
    for k = 2:length(tsim)
        tt = tic;
        % refresh trajectory ----------------------------------------------
        xlim(axTraj,[xCart(k)-10 xCart(k)+60]);
        ylim(axTraj,[-10 60]);
        set(trajPlane,  'XData', xPlane(1:k));
        set(trajPlane,  'YData',-zPlane(1:k));
        set(trajCart,   'XData', xCart(1:k));
        set(trajCart,   'YData', zCart(1:k));
        set(Plane,      'XData', xPlane(k));
        set(Plane,      'YData',-zPlane(k));        
        set(Cart,       'XData', xCart(k));
        set(Cart,       'YData', zCart(k));       
        set(cart_cable, 'XData', [xCart(k), xNodes(k,1)]);
        set(cart_cable, 'YData', [zCart(k), -zNodes(k,1)]);
        set(PzTraj   , 'XData', xPlane(1:k));
        set(PzTraj   , 'YData', pzMin(1:k));
        for i=1:N-1
          set(cable_handles(i), 'XData', [xNodes(k,i), xNodes(k,i+1)]);
          set(cable_handles(i), 'YData', [-zNodes(k,i), -zNodes(k,i+1)]);
        end
        %set(plane_cable, 'XData', [xNodes(k,end), xPlane(k)]);
        %set(plane_cable, 'YData', [-zNodes(k,end), -zPlane(k)]);
        %% refresh angle of attack ------------------------------------------
        set(trajAlpha,  'XData', tsim(1:k));
        set(trajAlpha,  'YData', rad2deg(Asim(1:k)));
        %% refresh velocity
        set(traVxPlane, 'XData', tsim(1:k));
        set(traVxPlane, 'YData', vxPlane(1:k));
        
        set(traVzPlane, 'XData', tsim(1:k));
        set(traVzPlane, 'YData', vzPlane(1:k));
        
        set(traVxCart,  'XData', tsim(1:k));
        set(traVxCart , 'YData', vxCart(1:k)); 
        %% refresh Tension
        set(trajTension, 'XData', tsim(1:k));
        set(trajTension, 'YData', Tsim(1:k));
          
        pause(tsim(k)-tsim(k-1)-2*toc(tt))
        drawnow
    end
end

