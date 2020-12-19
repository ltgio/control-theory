% Title:  parameter sensitivity analysis via rootlocus
clear all;close all;clc;

% assume a spring-damper-mass
% G(s) =     1
%       -------------
%       m s^2 + b s + k
% with m = k = 1;
% for define the parameter sensivity of b w.r.t damping ration we follow:
% 1) make everything in the form 1 + k*G(s) where k = b
% 2) G(s) = s/(s^2+1)

s = tf('s');
G = s/(s^2+1);
b = logspace(0,1.5,1000);
rlocus(G)