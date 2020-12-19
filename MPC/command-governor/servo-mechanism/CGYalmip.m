function g=CG(r,x,g,T,Hc,Phi,G,L,k0,dimRef,Psi,dimCon,delta,dimSys)
x
ops=sdpsettings('verbose',0);
W=sdpvar(dimRef,1);
V1=[];
for n=0:k0
%    Rkl=zeros(length(Phi),dimRef);
%         for j=0:n-1
%             Rkl=Rkl+(Phi^j)*G;
%         end
%         
% RKC=Hc*Rkl+L;
x=Phi*x+G*W;
V1=[V1 T*Hc*x+T*L*W<=g];
end 
dt=[];
for n=1:length(g)
    dt=[dt; norm(T(n,:),2)];
end 
V2= T*(Hc/((eye(dimSys)-Phi))*G+L)*W<=g-delta*dt;
V=[V1,V2];
solvesdp(V,(W-r)'*Psi*(W-r), ops)
g = double(W)