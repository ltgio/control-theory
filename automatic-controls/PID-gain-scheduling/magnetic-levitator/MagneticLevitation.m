function [Fode,ode,y,x,u,nx,nu] = MagneticLevitation
    %% Get Magnetic Levitation System [symbolic]
    % Equilibrium point
    % ueq = 5;
    % xeq = [sqrt(Km/(m*g0))*(ueq/R);0;ueq/R];
    import casadi.*
    
    nx = 3;
    nu = 1;

    g0 = 9.81;  R = 50;    L = 0.5;
    m  = 0.02; Km = 19.62; b = 0.1;
    
    x  = MX.sym('x',nx);      
    x1 = x(1); % position [m]
    x2 = x(2); % velocity [m/s]
    x3 = x(3); % current  [A]

    u  = MX.sym('u' ,nu); % voltage  [V]

    ode = [x2;
          g0-(b/m)*x2-(Km/m)*((x3/x1)^2);
         (1/(L+Km/x1))*(-R*x3+Km*(x2/(x1^2))+u)];  

    Fode = Function('ode',{x,u},{ode},char('x','u'),char('ode'));
    Fode.expand();
    y    = x1;  % measure position
    disp('Magnetic Levitation system created')
end