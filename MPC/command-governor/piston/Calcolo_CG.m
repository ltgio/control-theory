

function [cg]= Calcolo_CG(r,stato,Fhi,G,Hc,L,T,g,delta,ksegnato)

global rif
global VINCOLO_A
global VINCOLO_B
global inc_c
rif=r;

%    Questa funzione calcola il CG ad ogni passo

%    La sintassi è[cg]=(Fhi,G,Hc,L,T,g,r,delta,timeexp,TS,ksegnato);

% - 'Fhi,G,Hc,L' -  sono le matrici del primo anello stabilizzato;
% - 'T' - è la matrice dei vincoli sulle uscite;
% - 'g' - è il vettore dei vincoli;
% - 'r' - è la traccia temporale rappresentate il riferimento da inseguire;
% - 'delta' - è la tolleranza ammessa sul soddisfacimento dei vincoli;


% CODICE PER IL CALCOLO DEL COMMAND - GOVERNOR AD OGNI PASSO DI DURATA TS


% Inizio costruzione dei vincoli Per il calcolo del command - governor ad
% ogni passo di durata TS


if isempty(VINCOLO_A)
    
    I=eye(length(Fhi));
    nrT=size(T,1);
    aaa_1=zeros(nrT,1);
    aaa_2=T';
    
    for m=1:nrT
        aaa_1(m,1)=sqrt(T(m,:)*aaa_2(:,m));
    end
    
    vett_sqrt=aaa_1;
    
    v_hor=length(0:ksegnato);
    VINCOLO_A=zeros(size(T,1)*v_hor+size(T,1),size(Fhi,2)+size(L,2));
    VINCOLO_B=zeros(size(g,1)*v_hor+size(g,1),size(g,2));
    inc_c=1;
    inc_r=1;
    inc_rf=size(T,1);
    inc=0;
    temp=zeros(size(Fhi,1),size(G,2));
    cont=1;
    espo=1;
       
    VINCOLO_A(1:size(T,1),1:size(Fhi,2))= zeros(size(T,1),size(Fhi,2));              % Corrisponde al vincolo su w
    VINCOLO_A(1:size(T,1),size(Fhi,2)+inc_c:end)= T*((Hc*(inv(I-Fhi))*G)+ L);        % Corrisponde al vincolo su w
    VINCOLO_B(1:size(T,1),:)=g-delta*vett_sqrt;                                      % Corrisponde al vincolo su w
    
    VINCOLO_A((size(T,1))+inc_r:(size(T,1))+inc_rf,1:size(Fhi,2))= T*Hc*(Fhi^0);     % Corrisponde a k = 0
    VINCOLO_A((size(T,1))+inc_r:(size(T,1))+inc_rf,size(Fhi,2)+inc_c:end)=T*L;       % Corrisponde a k = 0
    VINCOLO_B((size(T,1))+inc_r:(size(T,1))+inc_rf,:)=g;
    
    % Costruzione di Rk
       
    for i=0:ksegnato-1
        temp = temp + (Fhi^i)*G;
        Rk(1+inc:size(Fhi,1)+inc,:)=temp;
        inc=inc+size(Fhi,1);
        
    end
    
    inc_rf=inc_rf+size(T,1);
    inc_r=size(T,1);
    inc=0;
    inc_c=1;
    espo=1;
    cont=1;
    
    for t=1:ksegnato
        
        VINCOLO_A((size(T,1))+inc_r+cont:(size(T,1))+inc_rf,1:size(Fhi,2))=T*Hc*(Fhi^espo);
        VINCOLO_A((size(T,1))+inc_r+cont:(size(T,1))+inc_rf,size(Fhi,2)+inc_c:end)=T*(Hc*Rk(1+inc:size(Fhi,1)+inc,:)+L);
        VINCOLO_B((size(T,1))+inc_r+cont:(size(T,1))+inc_rf,:)=g;
        
        inc_r=inc_r+size(T,1);
        inc_rf=inc_rf+size(T,1);
        espo= espo+1;
        inc=inc+size(Fhi,1);
        
    end
end

VINCOLO_A_A=VINCOLO_A(:,size(Fhi,2)+inc_c:end);
VINCOLO_B=VINCOLO_B;
VINCOLO_B_A=VINCOLO_A(:,1:size(Fhi,2));


% INIZIO CALCOLO DEL COMMAND - GOVERNOR AD OGNI PASSO DI DURATA TS = 0.01

options_1= optimset('TolX',0.000000001,'Display','Off') ;

succ_stato=stato';
w_succ=0;
VINCOLO_B_AGG=VINCOLO_B-VINCOLO_B_A*succ_stato';
[w_succ,FOBB_succ,EXITFLAG_succ,OUTPUT_succ,LAMBDA_succ,GRAD_succ,HESSIAN_succ] = fmincon(@Func_obj_succ,[w_succ],VINCOLO_A_A,VINCOLO_B_AGG,[],[],[],[],[],options_1);  
cg=w_succ;                   