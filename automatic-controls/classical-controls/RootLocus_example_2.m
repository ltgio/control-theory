% Title: Root-Locus example 1
clc;clear all;close all;

s   = tf('s');
num = (s+4);
den = s*(s+1)*(s+2)*(s+3);
GH  = num/den; 
rlocus(GH);grid on;hold on;

p  = pole(GH)  % show poles
z  = zero(GH)
k  = 1;
%% Let's plot the pzmap for the closed loop system as we sweep the gain k form 0 to 10
figure;
GH = zpk(z,p,k)
pzmap(GH);grid on;
for k = 1:10:101
    pzmap(feedback(GH * k,1)); % I assumed the GH in the feedforward and 1 as feedback 
    hold on
end
grid on;

