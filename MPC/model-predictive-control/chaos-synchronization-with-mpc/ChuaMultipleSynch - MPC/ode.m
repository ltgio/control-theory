function dx = ode(t,x,u)   
    %% Master: Chua circuit poly  - Slave:  Lorenz System
    % master: chua's circuit poly
    % slave : Lorenz System
    
    p = 10;q = 100/7;                  % slave system parameter
    a = 28;b = 10;c = 8/3;             % master system parameter
    
    x1 = x(1);y1 = x(2);z1 = x(3);     % master sys states 
    x2 = x(4);y2 = x(5);z2 = x(6);     % slave sys states
    
    ux = u(1);uy = u(2); uz = u(3);    % control to slave system
    
    f_x1 = 2*x1^3 - x1/7;              % non linearity master sys
   
    dx = [p*(y1-f_x1);   % Chua poly : f1        
          x1-y1+z1;      % Chua poly : f2
          -q*y1;         % Chua poly : f3
          b*(y2-x2)+ux;     % Lorenz : f1 
          x2*(a-z2)-y2+uy;  % Lorenz : f2
          x2*y2-c*z2+uz     % Lorenz : f3
          ];                
end
