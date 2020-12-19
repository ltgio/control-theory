function [xeq,ueq] = trim(dx,w,w0,wmin,wmax,nx)
    import casadi.*
    %% Trim model 
    Fw     = dx'*dx;         
    nlp    = struct('x',w, 'f',Fw);
    solver = nlpsol('solver','ipopt', nlp);
    sol    = solver('x0', w0, 'lbx', wmin, 'ubx', wmax); % trim

    if(full(sol.f)<eps)
        disp('----------------------------------------------------------');
        disp('Equilibrium Point Found');
        w_opt = full(sol.x);     
        xeq   = round(w_opt(1:nx),3);
        ueq   = round(w_opt(nx+1:end),3);
        disp(['xeq = [',num2str(xeq'),']']);
        disp(['ueq = [',num2str(ueq'),']']);
    else
        assert('Infeable problem')
    end
end
