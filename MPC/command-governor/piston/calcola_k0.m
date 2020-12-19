 function k0=calcola_k0(T,Hc,Phi,G,L,g,delta)
%calcolo struttura W
inv_I_Fi=pinv(eye(size(Phi))-Phi);
TT_compute=[];
    for i=1:size(T,1)
        TT_compute=[TT_compute;(sqrt(T(i,:)*T(i,:)'))];
    end
TT=TT_compute;
%fine calcolo struttura W

kmin=100;
kmax=400;
k0=kmax;

%calcolo struttura V(x)
for horizon=kmin:kmax
     vector_g=repmat(g,horizon,1); %vector g fino a k-1 (horizon +1 era il numero completo)
     T_times_Rkc_vect_compute=[];
     first_term_vect_compute=[]; %first_term=T*Hc*(Fi)^i;

%calcolo V(x) fino a k-1 
for k=0:horizon-1

   % R_k^x
    Rkx=zeros(size(Phi,1),size(G,2));
    for i=0:k-1
        Rkx=Rkx+((Phi^i)*G);
    end
   % fine R_k^x
                       
   % R_k^c
   Rkc=(Hc*Rkx)+L;
   % fine R_k^c
   T_times_Rkc_vect_compute=[T_times_Rkc_vect_compute;T*Rkc];
   first_term_vect_compute=[first_term_vect_compute;T*Hc*(Phi)^k];
end

T_times_Rkc_vect=T_times_Rkc_vect_compute;
first_term_vect=first_term_vect_compute;
%fine calcolo V(x) fino a k-1 

%calcolo ultimo elemento di V(x) a k
k=horizon+1;
%R_k^x
Rkx=zeros(size(Phi,1),size(G,2));
for i=0:k-1
    Rkx=Rkx+((Phi^i)*G);
end
% fine R_k^x

% R_k^c
T_times_Rkc=T*((Hc*Rkx)+L);
first_term=T*Hc*(Phi)^k;
% fine R_k^c                
%fine calcolo ultimo elemento di V(x) a k
%fine calcolo V(x)
buono=true;
for j=1:size(T,1)
    x=sdpvar(size(Phi,1),1);
    w=sdpvar(size(G,2),1);
    V1=[];
    V2=[];
    %%PRIMO VINCOLO%%
    V1=V1+(first_term_vect*x+T_times_Rkc_vect*w<=vector_g);
    %%PRIMO VINCOLO fine%%
                    
    %%SECONDO VINCOLO%%
    V2=V2+(T*(Hc*inv_I_Fi*G+L)*w<=g-delta*TT);
    %%SECONDO VINCOLO fine%%
                    
    %%VINCOLI TOTALI%%
    Vinc=V1+V2;
    %%VINCOLI TOTALI fine%%
             
    options=sdpsettings('verbose',0);
    OBJ=first_term(j,:)*x+T_times_Rkc(j,:)*w;
    solution=solvesdp(Vinc,-OBJ,options);
    if solution.problem~=0
          fprintf('errore ottimizzazione\n\n')
          solution
          error('errore')
    end
    max_j=-double(-OBJ)-g;
    if max_j>0
        buono=false;
        break
    end
end
if buono==true
   k0=horizon;
   fprintf('k0= %i\n', k0)
   break
   else
   fprintf('k0=k0+1 =%i\n', horizon+1)
   end
                
end

end
