%% Title: Fondamenti di Automatica pg:
clc;clear all;close all;

% Let's the Siso System 
s = tf('s');
G = 2/[(1+0.2*s)*(1+0.1*s)]

%% Step 1: check if it's stable
pole(G) % if no stabilize with a gain via root-locus

%% Step 2: satisfy static requirements
% 2.1: e0 = 0; step error = 0
% we need to add an integrator
% apply final value theorem
C1 = 5/s;
G1 = C1*G;
figure;step(feedback(G1,1));hold all;

% 2.2: e1 = 0.1; ramp error = 0.1
% apply final value theorem
C2 = 10/s;
G2 = C2*G;
figure;step(feedback(G2,1));

% the system is not intenally stable

%% Step 3: satisfy the dynamic requirements [frequency Domain]
T = G*C2 / (1 + G*C2);  % complemetary sensitivity 
S = 1/(1 + G*C2);       % sensitivity

figure;
bode(G*C2);hold on;grid on;
% bode(T);
% bode(S);

[Gm,Pm,Wgm,Wpm] = margin(G*C2) 

%%
Glag = (1+40/9*s)/(1+40*s);


%% Generate Lead Compensator
PhiMax = 40
a      = (1 + sin(deg2rad(PhiMax)))/(1 - sin(deg2rad(PhiMax)))
tau    = 1/(Wpm*sqrt(a));
Clead  = (a*tau*s + 1)/(tau*s + 1);

Clead = (1+1/7*s)/(1 + 1/105*s);

figure;bode(Clead)

[Gm,Pm,Wgm,Wpm] = margin(G*C2*Clead) 

T = G*C2*Clead / (1 + G*C2*Clead);  % complemetary sensitivity 
S = 1/(1 + G*C2*Clead);       % sensitivity

figure;step(T)

figure;
bode(G*C2);hold on;grid on;
bode(G*C2*Clead);

%%
figure;
bode(G*C2);hold on;grid on;
bode(G*C2*Glag);
bode(G*C1*Clead);

figure;grid on;hold on;
%step(feedback(G*C2,1));
step(feedback(G*C2*Glag,1));
step(feedback(G*C1*Clead,1));
%% Use of SisoTool
% sisotool(G)
