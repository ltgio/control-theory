function x_next = RK4(x0, mu, dynamics, h)
    
    % Input:    x0          - initial state
    %           mu          - stiffness parameter
    %           dynamics    - function describing the dynamics
    %           h           - step-size
    %
    % Output:   x_next      - numerical approximation of x(h)   
    
    k1     = dynamics(x0,            mu);
    k2     = dynamics(x0 + 1/2*h*k1, mu);
    k3     = dynamics(x0 + 1/2*h*k2, mu);
    k4     = dynamics(x0 + h*k3,     mu);
    x_next = x0 + h/6*(k1 + 2*k2 + 2*k3 + k4);   
end


