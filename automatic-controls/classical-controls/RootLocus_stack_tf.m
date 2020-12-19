%% Title: 
clc;clear all;close all;

s = tf('s');
% let's create different tf GH
GH1 = 1/(s*(s+4)*(s^2 + 4*s + 20));
GH2 = 1/(s*(s+4)*(s^2 + 5*s + 20));
GH3 = 1/(s*(s+4)*(s^2 + 6*s + 20));
GH4 = 1/(s*(s+4)*(s^2 + 6*s + 10));

% Let's stack togheter the GHs
GHstacked = stack(1,GH1,GH2,GH3,GH4);

% now we can feed to sisotool
sisotool(GHstacked)

% use this option if you want to design a signle compensator that works for
% all the transfert function i.e. for robust compensator