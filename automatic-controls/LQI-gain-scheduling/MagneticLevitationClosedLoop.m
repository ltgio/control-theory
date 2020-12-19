function [FodeLQI,odeLQI,Output,xa,r,na,nr] = MagneticLevitationClosedLoop(Kff,Kfb,xeq,ueq)
    %% Get Magnetic Levitation System [symbolic]
    % Equilibrium point
    % ueq = 5;
    % xeq = [sqrt(Km/(m*g0))*(ueq/R);0;ueq/R];
    import casadi.*
    
    nx = 3;
    na = nx+1; % number of augmented state 
    nr = 1;    % number of reference

    g0 = 9.81;  R = 50;    L = 0.5;
    m  = 0.02; Km = 19.62; b = 0.1;
    
    xa  = MX.sym('x',na);      
    x1 = xa(1); % position [m]
    x2 = xa(2); % velocity [m/s]
    x3 = xa(3); % current  [A]
    xe = xa(4); % xerror  
    r  = MX.sym('u' ,nr); % reference (position)[m]

    % get integrale state
    EffInteg = tf(1,[1 0]);
    sysInt   = ss(Kff*EffInteg);
    [Ae,Be,Ce,~] = ssdata(sysInt); % Controllable canonical form
    
    x  = [x1;x2;x3];
    u  = ueq - Kfb*(x-xeq) + Ce*xe;
    y  = x1;  % measure position
    
    % non linear model augmented with LQI regulator
    odeLQI = [x2;
             g0-(b/m)*x2-(Km/m)*((x3/x1)^2);
             (1/(L+Km/x1))*(-R*x3+Km*(x2/(x1^2))+u);
              Ae*xe + Be*(r-y)];  
    % outout augmeted with control u
    ya = [y; ueq - Kfb*(x-xeq) + Ce*xe];
    
    FodeLQI = Function('ode',{xa,r},{odeLQI},char('xa','r'),char('OdeNL_LQI'));
    FodeLQI.expand();
    
    Output = Function('ode',{xa},{ya},char('xa'),char('[y,u]'));
    
    disp('Magnetic Levitation system closed loop created')
    

end