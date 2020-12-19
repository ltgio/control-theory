function H = hessian_single(Z, lambda)                      
    global N R;
    H = [];
    for k = 1:N
        H = blkdiag(H,R);
    end
end