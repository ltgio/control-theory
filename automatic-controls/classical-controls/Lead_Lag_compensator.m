%% Title: Lead and Lag compensator effect
clc;clear all;close all;

s = tf('s');

sys = 50/(s*(0.25*s + 1))

lead = (0.088*s+1)/(0.022*s+1);
lag  = (10*s+1)/(100*s+1)

figure;hold all;
bode(sys);
bode(lead*sys);
bode(lag*sys);
grid on