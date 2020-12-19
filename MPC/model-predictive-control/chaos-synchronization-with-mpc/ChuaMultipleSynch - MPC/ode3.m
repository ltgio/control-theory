function dx = ode(t,x,u)   
    %% Master: Lorenz - Slave: Chua poly
    % master: Lorenz System
    % slave : chua's circuit poly
    sigma = 10;xi = 28;b = 8/3;        % master system parameter
    p = 10;q = 100/7;                  % slave system parameter
    
    x1 = x(1);y1 = x(2);z1 = x(3);     % master sys states 
    x2 = x(4);y2 = x(5);z2 = x(6);     % slave sys states
    
    ux = u(1);uy = u(2); uz = u(3);    % control to slave system
    
    f_x2 = 2*x2^3 - x2/7;              % non linearity slave sys
    
    dx = [sigma*(y1-x1);        % Lorenz f1 
          xi*x1-y1-x1*z1;       % Lorenz f2 
          x1*y1-b*z1;           % Lorenz f3 
          p*(y2-f_x2)+ux;       % Chua poly f1  
          x2-y2+z2+uy;          % Chua poly f2
          -q*y2+uz];            % Chua poly f3    
end
