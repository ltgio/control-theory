function [Phi,G,Hy,L] = makePrecompensatedPlant(sys,Gcs)
    % Make Plant  + LQI in the form 
    %    xdot(t)  = PHI*x(t) + G*g(t) + Gd*d(t)
    % [y(t);u(t)] = H*x(t) + L*d(t)
    
    sysOL = series(Gcs,sys,[1],[1]);
    sysCL = feedback(sysOL,1,[1],[1]);
    Hy    = [sysCL.C;zeros(1,size(sys.C,2)),Gcs.C]; % yCL = H*[y;u] PID/Compensator case 
 
    nh  = size(Hy,1);

    Phi = sysCL.a;
    G   = sysCL.b;
    L   = [sysCL.d; zeros(nh-1,1)];
end

