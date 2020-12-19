function x_next = rk4_step(ode_fun,x,u,h)                   
    k1 = ode_fun(0,x,u);
    k2 = ode_fun(0,x+h/2.*k1,u);
    k3 = ode_fun(0,x+h/2.*k2,u);
    k4 = ode_fun(0,x+h.*k3,u);
    x_next = x + h/6.*(k1+2*k2+2*k3+k4);
end