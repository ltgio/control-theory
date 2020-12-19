function x_next = IEU( x, mu, h, f, J)
% Implicit Euler integrator
max_it = 100;
x_next = zeros(2,1);
for i=1:max_it
    
    % Solve implicit nonlinear equation with the Newton's method
    res_J   = J(x,x_next,h,mu);
    J_m     = full(res_J); 
    res_r   = f(x,x_next,h,mu);
    r_m     = full(res_r);
    n_step  = J_m\r_m;
    norm(n_step);
    x_next  = x_next - J_m\r_m;
    
    % If convergence achieved ( norm(x_{i+1} = x_{i}) < accuracy )
    if norm(n_step) < 1.0e-8
        break
    end
end
if i ==max_it
    warning('Maximum number of Newton iterations reached.')
end
end

