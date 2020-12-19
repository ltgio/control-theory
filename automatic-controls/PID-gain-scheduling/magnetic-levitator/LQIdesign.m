function [K_fb,K_ff] = LQIdesign(sys,Q,R,nx,nu)
    [np,~] = size(sys.C);
    A_aug = [sys.A , zeros(nx,np); 
             sys.C , eye(np,np)];       
    B_aug = [sys.B ; zeros(np,nu)];
    C_aug = [zeros(np,nx),eye(np)];
    D_aug = zeros(np,nu);

    K_aug = lqr(A_aug, B_aug, Q, R);
    K_fb  = K_aug(:,1:nx);
    K_ff  = K_aug(:,nx+1:end);
    
    disp('Gain LQI')
    disp(['K_fb = ',num2str(K_fb)]); 
    disp(['K_ff = ',num2str(K_ff)]); 
end