function [ F_lq, f_int ] = LQI_control( SYS_TD,Ta,Ts,Riferimento)

[Ad,Bd,Cd,~]=ssdata(SYS_TD);

%matrice di peso dell'errore di inseguimento 
Q_epsilon=1;

dimA=size(Ad);
dimC=size(Cd);

% Sistema nello spazio aumentato
A_aug=[Ad zeros(dimA(1),1);Cd*Ad ones(dimC(1),1)];
B_aug=[Bd;Cd*Bd];
C_aug=[zeros(1,dimA(1)) 1];
D_aug=0;

% Calcolo rho
rho=exp(-3*Ts/Ta);
r=1/rho;

% Calcolo il guadagno di Kalman e l'azione integrale
[F_aug,~,~] =dlqry(r*A_aug,r*B_aug,C_aug,D_aug,Q_epsilon,rho);

F_lq = F_aug(1,1:dimA(1));
f_int = F_aug(1,dimA(1)+1);

x0=zeros(size(Ad,1),1);

%% Simulazione

N = 10/Ts;

rif=Riferimento;
% rif = 0.3*ones(1,N);

%definisco la condizione iniziale dell'integratore
epsilon=0;

x_pred=x0;
%collezione della traiettoria regolata del sistema
x_fin=[];
x_fin=[x_fin x0];
%collezione delle mosse di controllo
u_seq=[];


%creo le mosse di controllo ottime e l'evoluzione ottima dello stato
for i=1:N;
    u_Kalman=-F_lq*x_pred-f_int*epsilon;
    x_new=Ad*x_pred+Bd*u_Kalman;
    y=Cd*x_new;
    epsilon=epsilon+(y-rif(i));
    x_pred=x_new;
    x_fin=[x_fin x_new];
    u_seq=[u_seq u_Kalman];
end;

rif = [rif 0];

% grafico l'evoluzione dello stato del sistema
t=0:Ts:Ts*N;

figure(1)
subplot(2,2,1)
plot(t,x_fin(1,:));
title('Angolo asta')
grid;
subplot(2,2,2)
plot(t,x_fin(2,:));
title('Velocità angolare asta')
grid;
subplot(2,2,3)
plot(t,x_fin(3,:));
title('Altezza pistone') 
grid;
subplot(2,2,4)
plot(t,x_fin(4,:));
title('Velocità verticale pistone') 
grid;

%grafico l'uscita del sistema 
% figure(2)
% plot(t,Cd*x_fin);
% title('uscita controllata')
% 
%grafico il segnale di comando
figure(3)
t1=0:Ts:Ts*(N-1);
plot(t1,u_seq);
title('ingresso di controllo')

end

