function dx = ode_V3(t,x,u)      
    % uncertainty to slave system
    % nominal parameter: p = 10; q = 100/7;
    p = 10;    q = 100/7;                                            
    ps = 10+1; qs = 100/7-1;  % Vertex3
    
    x1 = x(1); y1 = x(2); z1 = x(3);
    x2 = x(4); y2 = x(5); z2 = x(6);
    ux = u(1); uy = u(2); uz = u(3);
    
    f_x1 = 2*x1^3 - x1/7;
    f_x2 = 2*x2^3 - x2/7;
    
    dx = [p*(y1-f_x1); 
          x1-y1+z1;
          -q*y1;          
          ps*(y2 - f_x2) + ux;
          x2 - y2 + z2 + uy;
          -qs*y2 + uz;
          ];  
end