clear all;
close all;
clc;

% MODIFICHE FATTE:
% #1 Eliminata esecuzione ciclica del comando jacobian;
% #2 Isolate le function per trovare stato di equilibrio e linearizzazione;
% #3 Metodi getLinearise e getEquilibrium resi indipendenti dal modello
% preso in esempio;
% #4 Function getModel restituisce il modello da funzione 'ode';
% #5 getDimension restituisce le dimensioni del modello scritto in 'ode'.
%**************************************************************************
% 
% > Creata struttura denominata 'modello', con i seguenti campi:
%   > state: vettore di stato (symbolic);
%   > input: vettore di ingresso (symbolic);
%   > p: vettore dei parametri;
%   > non_lin: modello non lineare ottenuto usando la function getModel;
%   > EquilibriumInput: vettore degli ingressi di equilibrio (define by
%   user);
%   > EquilibriumState: vettore dello stato di equilibrio ottenuto dalla
%   function getEquilibrium;
%   > InitialCondition: vettore condizioni iniziali (define by user);
%   > statespace: object SYS representing the continuous-time state-space
%   model (see 'ss');
%   > eigenvalues: self-evident!
%   > TransferFunction: as above! 

% PER USARE LE FUNCTION
% Si possono usare getEquilibrium, getLinearise e getModel cambiando 
% solamente la funzione ode definisce il modello dell'impanto
% 


%% Define model and all symbols 
% Define state and input vectors

[nx, nu] = getDimension();

x       = sym('x%d',[nx,1]);
u       = sym('u%d',[nu,1]);

modello.state     = x;
modello.input     = u;

%% Define parameters

load parameters;

m         = parameters.massPlane;
rho       = parameters.airDensity;  
g         = parameters.gravity;
Sref      = parameters.wingArea;                                   
liftCoeff = parameters.liftCoefficient_CL0_CLa;
dragCoeff = parameters.dragCoefficient_CD0_CDa_CDa2;

% Parameter vector     
modello.p = [m, rho, g, Sref, liftCoeff, dragCoeff]; 

%% Equilibrium point

modello.non_lin = getModel(x, u, modello.p);

% Define the equilibrium input and initial state
modello.EquilibriumInput = deg2rad(5);
modello.InitialCondition = [0,0,10,0];

% Get equilibrium state vector

modello.EquilibriumState = getEquilibrium(modello);

%% Symbolic linearization via Jacobian function

[A,B,C,D] = getLinearise(modello);

modello.statespace          = ss(A,B,C,D);
modello.eigenvalues         = eig(A);
modello.TransferFunction    = tf(modello.statespace);

%% Simulation

modello.time = 10;
[t, x_ode45] = ode45(@(t,x) ode(t,x,modello.EquilibriumInput,modello.p),...
    [0 modello.time], modello.InitialCondition);
plot(t, x_ode45);
grid on;



