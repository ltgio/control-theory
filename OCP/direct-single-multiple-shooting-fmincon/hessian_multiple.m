function H = hessian_multiple(x, lambda)                    
    % NOTE: this is a simple Gauss-Newton Hessian approximation to improve 
    % the convergence behaviour of fmincon

    global N Q R;
    H = [];
    for k = 1:N
        H = blkdiag(H,Q,R);
    end
    H = blkdiag(H,Q);
end