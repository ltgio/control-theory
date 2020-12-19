% Title: Root-Locus example 1
clc;clear all;close all;

s   = tf('s');
num = 1;
den = s*(s+4)*(s^2 + 4*s + 20);
GH  = num/den; 
rlocus(GH);grid on

% alternative way 1
GH = tf([1],[1 8 36 80 0])
p  = roots([1 8 36 80 0]) % roots of polynomial
p  = pole(GH);             % show poles
z  = zero(GH);
k  = 1; % gain

% alternative way 2
GH = zpk(z,p,k)
pzmap(GH);grid on;

%% Let's plot the pzmap for the closed loop system as we sweep the gain k form 0 to 10
for k = 1:10:101
    pzmap(feedback(GH * k,1)); % I assumed the GH in the feedforward and 1 as feedback 
    hold on
end
grid on;

% you can use siso tool
%sisotool(GH)