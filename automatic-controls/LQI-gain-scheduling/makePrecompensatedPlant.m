function [Phi,G,H,L] = makePrecompensatedPlant(sys,K_fb,K_ff)
    % Make Plant  + LQI in the form 
    %    xdot(t)  = PHI*x(t) + G*g(t)+Gd*d(t)
    % [y(t);u(t)] = H*x(t) + L*d(t)
    
    EffInteg = tf(1,[1 0]);
    sysInt   = ss(K_ff*EffInteg);
    [~,~,Ci,~] = ssdata(sysInt); % Controllable canonical form
    
    sysFb = ss(sys.A-sys.B*K_fb,sys.B,sys.C,sys.D);
    sysSe = series(sysInt, sysFb,1,1); 
    sysCL = feedback(sysSe,1,1,1);

    %H  = [sysCL.C;zeros(length(1),nx),Ci]; % yCL = H*[y;u] PID/Compensator case
    H   = [sysCL.C;-K_fb,Ci]; % yCL = H*[y;u] LQI case
    nh  = size(H,1);

    Phi = sysCL.a;
    G   = sysCL.b;
    L   = [sysCL.d; zeros(nh-1,1)];
end

