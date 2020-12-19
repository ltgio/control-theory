%%%%%%% This script loads needed variable in the workspace and starts the simulation.
%close all;clear all;clc;
%%%%%%%%%%%% LOAD SYSTEM PARAMETERS %%%%%%%%
makeSystem;
[W q b k0] = CGOffLineSetting(Phi,G,Gd,Hc,L,Ld,g,dmax,delta,epsilon,dimRef,dimCon,dimSys,T);
k0    = 10;
delta = 0.000001;
%%%%%%%%%%%% SIMULATION %%%%%%%%%%%%%%%%%%%
% open('simulator')
sim('simulator')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% xc = zeros(1,4);
% xp = zeros(1,4);
% 
% g1 = CG(reference,[xc,xp]',W,q,b,T,Hc,Phi,G,L,k0,dimRef,Psi)
% %g2 = CGCasadi(reference,[xc,xp]',W,q,b,T,Hc,Phi,G,L,k0,dimRef,Psi)
% g3 = CGYalmip(reference,[xc,xp]',g,T,Hc,Phi,G,L,k0,dimRef,Psi,dimCon,delta,dimSys)
% 

%%%%%%%% PLOTTING %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% This part is customable depending on system structure
figure(1)
f2Deg=180/pi;
%plot(t,f2Deg*y,'k',t,f2Deg*y1,'b-.',t,f2Deg*[r; r(1)*ones(length(t)-length(r),1)],'r-.','LineWidth',2);
%legend('System with CG','System without CG','Reference')

grid
xlabel('Time(s)')
title('Load position (deg)')

figure(2)
plot(t,c1,'k',t,c1_1,'b-.',t,78.5398*ones(length(t),1),'r-.',t,-78.5398*ones(length(t),1),'r-.','LineWidth',2);
grid
xlabel('Time(s)')
title('Torsional Torque (Nm)')
legend('System with Predictive Reference','System without Predictive Reference','Costrainsts boundary')

figure(3)
plot(t,[c2; zeros(length(t)-length(c2),1)],'k',t,[c2_1; zeros(length(t)-length(c2),1)],'b-.',t,220*ones(length(t),1),'r-.',t,-220*ones(length(t),1),'r-.','LineWidth',2);
grid
xlabel('Time(s)')
title('Input Voltage (V)')
legend('System with Predictive Reference','System without Predictive Reference','Costrainsts boundary')

figure(4)
plot(t,f2Deg*[gt; gt(length(gt))*ones(length(t)-length(gt),1)],'k',t,f2Deg*[r; r(1)*ones(length(t)-length(r),1)],'b-.','LineWidth',2);
grid
xlabel('Time(s)')
title('Reference (deg)')
legend('Predictive Reference','Desiderated Reference')
