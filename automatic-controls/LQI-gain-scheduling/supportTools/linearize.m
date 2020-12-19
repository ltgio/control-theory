function [A,B,C,D,infoR,infoO] = linearize(dx,y,x,u,xeq,ueq)
    import casadi.*
        
    A_fun = Function('A',{x,u},{jacobian(dx,x)},char('x','u'), char('A'));
    B_fun = Function('B',{x,u},{jacobian(dx,u)},char('x','u'), char('B'));
    C_fun = Function('C',{x,u},{jacobian(y,x)} ,char('x','u'), char('C'));
    D_fun = Function('D',{x,u},{jacobian(y,u)} ,char('x','u'), char('D'));
    
    A     = full(A_fun(xeq,ueq));
    B     = full(B_fun(xeq,ueq));
    C     = full(C_fun(xeq,ueq));
    D     = full(D_fun(xeq,ueq));
    disp('----------------------------------------------------------');
    %% check stability
    disp('Eigenvalues')
    checkStability = eig(A);
    for i=1:length(A)
        disp(['lambda',num2str(i),' = [',num2str(checkStability(i)),']']); 
    end 
    disp('----------------------------------------------------------');
    %% check Controllability
    R = ctrb(A,B);
    unco = length(A) - rank(R); % Determine the number of uncontrollable states.
    if unco == 0
        infoR = 'it is fully Controllable';
        disp(infoR)
    else
       infoR = ['System has n = ',num2str(unco),'uncontrollable states'];
       disp(infoR); 
    end
    disp('----------------------------------------------------------');
    %% check observability
    O = obsv(A,C);
    unco = length(A) - rank(O); % Determine the number of uncontrollable states.
    if unco == 0
        infoO = 'it is fully observable';
        disp(infoO)
    else
        infoO = ['System has n = ',num2str(unco),'unobservable states'];
        disp(infoO); 
    end
    disp('----------------------------------------------------------');
end
