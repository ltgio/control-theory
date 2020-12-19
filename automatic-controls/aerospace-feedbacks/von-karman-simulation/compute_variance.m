%% Simulation for low altitude
clear;close all;clc;
load init                  %load bus of turbulence parameters
Vcruise        = 30; 
Vc_min         = -5;
Vc_max         = 5;
time           = 60;  
exp            = 100;
altitude_worst = 10;
for i=1:exp
sim Turbulence_VonKarman
varianceV_lowTurb(i,:) = var(v_low.signals.values);
varianceV_medTurb(i,:) = var(v_med.signals.values);
varianceV_sevTurb(i,:) = var(v_high.signals.values);
varianceW_lowTurb(i,:) = var(w_low.signals.values);
varianceW_medTurb(i,:) = var(w_med.signals.values);
varianceW_sevTurb(i,:) = var(w_high.signals.values);
end
varianceV_lowTurb_10m = mean(varianceV_lowTurb)
varianceV_medTurb_10m = mean(varianceV_medTurb)
varianceV_sevTurb_10m = mean(varianceV_sevTurb)
varianceW_lowTurb_10m = mean(varianceW_lowTurb)
varianceW_medTurb_10m = mean(varianceW_medTurb)
varianceW_sevTurb_10m = mean(varianceW_sevTurb)
% 300 meters
altitude_worst = 300;
for i=1:exp
sim Turbulence_VonKarman
varianceV_lowTurb(i,:) = var(v_low.signals.values);
varianceV_medTurb(i,:) = var(v_med.signals.values);
varianceV_sevTurb(i,:) = var(v_high.signals.values);
varianceW_lowTurb(i,:) = var(w_low.signals.values);
varianceW_medTurb(i,:) = var(w_med.signals.values);
varianceW_sevTurb(i,:) = var(w_high.signals.values);
end
varianceV_lowTurb_300m = mean(varianceV_lowTurb)
varianceV_medTurb_300m = mean(varianceV_medTurb)
varianceV_sevTurb_300m = mean(varianceV_sevTurb)
varianceW_lowTurb_300m = mean(varianceW_lowTurb)
varianceW_medTurb_300m = mean(varianceW_medTurb)
varianceW_sevTurb_300m = mean(varianceW_sevTurb)
% 150 meters
altitude_worst = 150;
for i=1:exp
sim Turbulence_VonKarman
varianceV_lowTurb(i,:) = var(v_low.signals.values);
varianceV_medTurb(i,:) = var(v_med.signals.values);
varianceV_sevTurb(i,:) = var(v_high.signals.values);
varianceW_lowTurb(i,:) = var(w_low.signals.values);
varianceW_medTurb(i,:) = var(w_med.signals.values);
varianceW_sevTurb(i,:) = var(w_high.signals.values);
end
varianceV_lowTurb_150m = mean(varianceV_lowTurb)
varianceV_medTurb_150m = mean(varianceV_medTurb)
varianceV_sevTurb_150m = mean(varianceV_sevTurb)
varianceW_lowTurb_150m = mean(varianceW_lowTurb)
varianceW_medTurb_150m = mean(varianceW_medTurb)
varianceW_sevTurb_150m = mean(varianceW_sevTurb)