clear;close all;clc;
load init                  %load bus of turbulence parameters
Vcruise   = 30; 
Vc_min    = -5;
Vc_max    = 5;
time      = 60;           %simulation for 5 minutes
altitude_worst=150;

sim Turbulence_VonKarman
