clear, close all;
clc;

%% 

Ts = 0.01;
tfin = 10;

%% Paramentri

par = [1 5 1.5 4 20 0.5 1 0.5];
m1 = par(1);
k1 = par(2);
b1 = par(3);
m2 = par(4);
k2 = par(5);
b2 = par(6);
 L = par(7);
 R = par(8);

g = 9.81;

% k2 = (-k1*L*m1 + k1*m2*R )/ (L*m1*R^2);

%% Linearizzazione del sistema

xeq = [0 0 -g*L*m1*R/k1 0];
heq = xeq(3);
ueq = m2*g + (k2 + k1/R^2)*heq;

x0 = [0 0 0 0]';

[A,B,~,~] = linmod('dyn_model',xeq, ueq);
C = [1 0 0 0];
D = 0;

SYS_TC = ss(A,B,C,D);
dimA = size(A,1);
dimB = size(B,2);
dimC = size(C,1);
F = zpk(SYS_TC);

%% Discretizzazione

SYS_TD = c2d(SYS_TC,Ts);
[Ad,Bd,Cd,Dd] = ssdata(SYS_TD);

%% Controllo Integrale

% Specifica sul tempo di assestamento
Ta = 0.5;
% Carico segnale di riferimento
load ref;
[F_lq,f_int] = LQI_control(SYS_TD,Ta,Ts,Riferimento);
close all;

sysInt = ss(tf(f_int, [1 -1], Ts));
[Ai,Bi,Ci,Di] = ssdata(sysInt);

% Sistema retroazionato
sysFeedback = ss(Ad-Bd*F_lq,Bd,Cd,Dd,Ts);

sysEsteso = series(sysInt, sysFeedback,1,1); 

sysTotal = feedback(sysEsteso,1,1,1);

% Matrice di segnatura delle variabili vincolate
Hc = [1 0 0 0 0; 0 1 0 0 0; -F_lq Ci];
nc = size(Hc,1);

% Estraggo matici per il CG, Fi - G - Hy - Hc - L
Fhi = sysTotal.a;
G = sysTotal.b;
Hy = sysTotal.c;
L = [sysTotal.d; zeros(nc-1,1)];

% Simulazione con controllo integrale
% simulazioneLQI;

% Valutazione della proprietà di offset free
% OffsetFree(FI, G, Hy)

%% Definizione dei vincoli 

qmin = -5;
qmax = 5;

hmin = - 0.15;
hmax = + 0.15; 

Fmin = -40;
Fmax = +40;

T = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
b = [qmax -qmin hmax -hmin Fmax -Fmin]';

% Tolleranza algoritmo CG
delta = 1e-3;

% Calcolo orizzonte di predizione
% k0 = calcola_k0(T, Hc, FI, G, L, b, delta);
k0 = 35;
% 
%% Simulazione Command Governor

Controllo_CG;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
