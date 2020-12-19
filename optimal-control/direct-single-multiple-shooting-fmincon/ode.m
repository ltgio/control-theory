function dx = ode(t,x,u)                                    

    p = x(1);
    theta = x(2);
    v = x(3);
    omega = x(4);
    F = u(1);
    
    M = 1;
    m = 0.1;
    g = 9.81;
    l = 0.8;

    dx = [   v; ...
             omega; ...
             -l*m*sin(theta)*omega^2 + F + g*m*cos(theta)*sin(theta)/(M + m - m*cos(theta)^2); ...
             -l*m*cos(theta)*sin(theta)*omega^2 + F*cos(theta) + g*m*sin(theta) + M*g*sin(theta)/(l*(M + m - m*cos(theta)^2)) ];

end