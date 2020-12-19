% In this script system parameters are specified.
% Several variables defined in this script appears in blocks in Simulink
% file simulator.mdl.
%Several variables defined are follewed by the symbol '%*'. 
%This indicates that you can't change 
%the name of this variable in order to avoid errors in Simulink simulation 
%because they appears in Simulink blocks in Phile simulator.mdl.


%%%%%%%%%%%%%%% PLANT PARAMETERS %%%%%%%%%%%%%%

Ls=1;
betaM=0.1;
kt=10;
kteta=1280.2;
betaL=25;
JM=0.5;
R=20;
p=20;
JL=20*JM;
Ts=0.1;%* %Sampling Time 

%%%%%%%%% CONTINOUS SYSTEM STATE SPACE MODEL%%%%%%%%%%%%%%%%%%%%%
A=[0 1 0 0;
   -kteta/JL -betaL/JL kteta/(p*JL) 0;
   0 0 0 1;%*
   kteta/(p*JM) 0 -kteta/(p^2*JM) -(betaM+kt^2/R)/JM];
B=[0;0;0;kt/(R*JM)];%*
C=[1 0 0 0
   kteta 0 -kteta/p 0];%*
D=[0 0
   0 0];%*


Bd=[0; 0; 0; 0]; % disturbance input matrix
SYSc1=ss(A,[B Bd],C,D);


%%%%%%%%% DISCRETIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SYSd1=c2d(SYSc1,Ts);
[Ad1,Bd1,Cd1,Dd1]=ssdata(SYSd1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%% DISCRETE CONTROL LAW %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

z=tf('z',Ts);
Gcz=1000*(9.7929*z^3-2.1860*z^2-7.2663*z+2.5556)/(10*z^4-2.7282*z^3-3.5585*z^2-1.3029*z-0.0853);
[Ac Bc Cc Dc]=ssdata(Gcz);%*

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%% GLOBAL PRECOMPENSATED SYSTEM %%%%%%%%%%%%%%%%%%%%%%
Plant1=(series(Gcz,SYSd1,[1],[1]));
sys2=feedback(Plant1,1,[1],[1]);
Phi=sys2.a;%*
G=sys2.b;%*
Gd=[0 0 0 0 0 0 0 0]';%*
C=[sys2.c;zeros(length(1),4) Cc];
D=[sys2.d;0]; 
Hy=C;%* %Segnature matrix
Hc=[Hy(2:3,:)];%* %Segnature matrix
L=[D(2:3,1)];%*
Ld=[0;0];%*
T = [1 0; 0 1;-1 0;0 -1];%*
g=[78.5398;220;78.5398;220];%* %Constraints region
g0=g;
dimSys=length(Phi);%* % system dimension
dimCon=length(g)/2;%* % vector constraints dimension
dimRef=1;%* % vector reference dimension
%%%%%%%%%%%%%   INITIAL CONDITION %%%%%%%%%%%%%
x0=zeros(length(Phi),1);%*

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

reference=pi/2;%* %reference



Psi=1;%* %weight matrix
epsilon=1e-8;
delta=1e-8;
dmax=0.01*1e-1*ones(1,1);%
