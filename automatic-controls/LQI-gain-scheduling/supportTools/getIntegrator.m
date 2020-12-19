function Integrator = getIntegrator(Fode,x,u,ts,M)
    import casadi.*
    dt = ts/M; 
    k1 = Fode(x,u);
    k2 = Fode(x+dt/2.0*k1,u);
    k3 = Fode(x+dt/2.0*k2,u);
    k4 = Fode(x+dt*k3    ,u);
    xf = x + dt/6.0*(k1+2*k2+2*k3+k4);
    % Create a function that simulates one step propagation in a sample
    RK4 = Function('RK4',{x,u},{xf});
    X = x;
    for i=1:M
        X = RK4(X,u);
    end
    % Create a function that simulates all step propagation on a sample
    Integrator = Function('one_sample',{x,u},{X},char('x(k)','u(k)'),char('x(k+1)'));
    % speedup trick: expand into scalar operations
    Integrator = Integrator.expand();
end
