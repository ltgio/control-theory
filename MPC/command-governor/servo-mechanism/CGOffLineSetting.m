% In this function CG off-line parameters are determined. 
% This is an usefull tool to compute several parameters needed to 
% CG function. 
% OUTPUT
%   AW gW  - matrix used for the steady-state contstraints region
%   inequalities AW<=gW
%   gk     - represents the Cinf region
%   k0     - Virtual Horizon
%INPUT
%   Phi     - dynamic matrix of the global precompensated system
%   G       - input command matrix of the global precompensated system
%   Gd      - input disturbance matrix of the global precompensated system
%   Hc      - state/costrained output matrix
%   L       - input/costrained output matrix
%   Ld      - disturbace/costrained output matrix
%   g       - costraints
%   delta   - margin of Cdelta region
%   epsilon - margin for Cinf region
%   dimRef  - reference dimension
%   dimCon  - constraints dimension
%   dimSys  - system order
%   T       - projection matrix


%%%%%%%%%%% DISTURBANCE PARAMETERS %%%%%%%%%%%%
function [W q b k0]=CGOffLineSettings(Phi,G,Gd,Hc,L,Ld,g,dmax,delta,epsilon,dimRef,dimCon,dimSys,T)
ke=0;
if dmax>0
    sigmaHc=max(svds(Hc));
    sigmaGd=max(svds(Gd));
    lambda=max(abs(eig(Phi)));
    if(lambda==1)
        lambda=0.98;
    end
    M=0.1;
    i0=1;
    eps=1e-50;
    while(1)
        found=1;
        for i=1:1000;
            if(norm(Phi^i)>M*lambda^i)
                found=0;                
                break
            end
        end
        if(found==0)
            M=M+1;
        else 
            break
        end
    end
    ke = (log(epsilon)+log(1-lambda)*sigmaHc*sigmaGd*M*dmax)/log(lambda);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g0=g;
%%%%%%%%%%%%%%%%%%% COMPUTATION OF Cdelta %%%%
for j=1:2*dimCon
      x0 = [zeros(length(dmax))]';
      A  = [eye(length(dmax));-eye(length(dmax))];
      B  = [dmax;-dmax];     
      [d,cdkj] = fmincon(@(x) (-(T(j,:)*Ld*x)), x0, A , B);
    g(j) = g(j)+cdkj;
end
gke=[g];%*
for i=1:ceil(ke)
    for j=1:2*dimCon
        [d cdkj]=fmincon(@(x)(-(T(j,:)*Hc*(Phi^(i-1))*Gd*x)),[zeros(length(dmax))]',[eye(length(dmax));-eye(length(dmax))],[dmax;-dmax]);
    g(j)=[g(j)+cdkj];
    end
    gke=[gke;g];
end
q = zeros(dimCon,1);%*
for i=1:length(T)
    q(i) = g(i)-(delta+epsilon)*sqrt(T(i,:)*T(i,:)');
end

W=T*(Hc*inv(eye(dimSys)-Phi)*G+[L]);%* %AW*w<=gW, steady-state constraints

%%%%%%%%%%%% K0 COMPUTATION %%%%%%%%%%%%%%%
matV=[];
k=1;%*
k0=0;%*

% First method %
eps=1e-3;
xt=0.1*ones(dimSys,1);
p0=norm(T*Hc*(Phi^1)*xt);
while(1)
    g01=g0;   
    if(norm((T*Hc*(Phi^k)*xt))<=eps*p0)  %%% vedi con il non lineare
        k0=k;
        break
    else
        k=k+1;
    end
end


% Second Method $
% while(1)
%    
%     Rkl=zeros(length(Phi),dimRef);
%     for j=0:k-1
%         Rkl=Rkl+(Phi^j)*G;
%     end
%     Rkc=Hc*Rkl+L;
% 
% 
%     Gk=zeros(2*dimCon,1);
%     for j=1:2*dimCon
%        gtemp1=g;
%        gtemp2=g;       
%        [xj Gkj]=fmincon(@(x)(-(T(j,:)*Hc*Phi^k*x(1:8)+T(j,:)*Rkc*x(9)-gtemp1(j))),[zeros(dimSys,1);zeros(dimRef,1)],[[T(j,:)*Hc*Phi^k T(j,:)*Rkc];[zeros(length(AW),dimSys) AW]],[gtemp2(j,:);g0]);
%        Gk(j)=-Gkj;
%         
%     end
%    
%     
%     
%     if(max(Gk)<=0)
%         k0=k;
%         break;
%     else
%         k=k+1;
%     end
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%





%%%%%%%%%%% COMPUTING TRANSIENT DISTURBANCE-FREE SET CONSTRAINT %%
if(ke==0)
    b=gke;%*
    for i=1:k0
        b=[gk;gke];
    end
else
    b=gke(1:2*dimCon*(k0+1));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

