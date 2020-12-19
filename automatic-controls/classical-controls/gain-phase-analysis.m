% analysis of margin gain and phase
clc;clear all;close all;
s = tf('s');

% open loop transfert function G(s)
G = 1.3*(s+2)/(s^3 + s^2 + 6*s +1)

% we have gain phase = 120 [deg] and gain phase = 11.7 
bode(G);grid on;
hold all
% however if we have some variation in the parameters e.g.
G = 1.3*(s+2)/(s^3 + s^2 + 3*s +1)
bode(G)
% this system has almost no margins
figure;
nyquist(G)