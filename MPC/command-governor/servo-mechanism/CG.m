%This function implements a one step CG action for a certain system
%described by Phi,G,Hc and L starting from an initial state xt forecasting
%virtual system evolution up to k0 steps. The solved problem is
%minimize (r-w)'Psi(r-w)
%s.t. W*w<=q steady state constraints
%     A*w<=b virtual constraints 
%OUTPUT
%   g       - the modified reference
%INPUT
%   r       - the desiderable reference
%   x       - the measured state at time t
%   W, q    - needed for test inequality As+ACo-w+ADwAPQ-bs
%   b       - represents the Cinf region
%   T       - projection matrix
%   Phi, G,  Hc, L  - matrices of state space model of the precompensated system
%   k0      - Virtual Horizon
%   dimRef  - reference vector dimension
%   Psi     - wheight matrix for objective function

function g=CG(r,x,W,q,b,T,Hc,Phi,G,L,k0,dimRef,Psi)
bt=[];
A=[];
[m,~]  =size(Hc);
len    = 2*m;

for i=1:k0       
   bt = [bt;b(i*len+1:(i+1)*len,:)-(T*Hc*(Phi^i)*x)];    
   Rkl=zeros(length(Phi),dimRef);
        for j=0:i-1
            Rkl=Rkl+(Phi^j)*G;
        end
    Rkc=Hc*Rkl+L;
    A=[A;T*Rkc];
end

term = [ bt ; q];
mat  = [A; W];
[p,fval,exit,output] = fmincon(@(w)((w-r)'*Psi*(w-r)),r,mat,term)
g=p;
end
