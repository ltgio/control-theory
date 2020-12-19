%% Title: Description of a Second order System
clc; clear all;close all;

% Parameters
w0    = 1;   % natural frequency [rad/s]
P0    = 15; % percentage overshoot [%]. Overshoot percentage is related to delta
delta = abs(log(P0/100)) / (sqrt(pi^2 + log(P0/100)^2)); % damping ration 0 <= delta<= 1

% Transfert Function ======================================================
s    = tf('s');
tf_second_order = w0^2/(s^2 + 2*delta*w0*s + w0^2);

step(tf_second_order)

% NB: the natural frequency w0 does not affect the overshoot
% prove
figure;
for w0 = 0.5:0.5:2
    tf_second_order = w0^2/(s^2 + 2*delta*w0*s + w0^2);
    step(tf_second_order)
    hold all
end

% Plot pole-zero of second order system
figure;
for w0 = 0.5:0.5:2
    tf_second_order = w0^2/(s^2 + 2*delta*w0*s + w0^2);
    pzmap(tf_second_order)
    hold all
end
grid on

%% State space ------------------------------------------------------------
A = [  0,         1;
     -w0^2, -2*delta*w0];
B = [0; w0^2];
C = [1,0];
D = 0;

sys_second_order = ss(A,B,C,D,'StateName',{'x1','x2'},...
                              'InputName',{'u'},...
                              'Name','Second Order System state-space');
figure;step(sys_second_order)

%% Root-locus -------------------------------------------------------------
[r,k] = rlocus(tf_second_order)
figure;rlocus(sys_second_order)


