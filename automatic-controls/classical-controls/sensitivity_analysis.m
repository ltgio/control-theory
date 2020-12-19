clc;clear all;close all;

s = tf('s');
CG = 0.38*(s^2 + 0.1*s + 0.55)/(s*(s+1)*(s^2 + 0.06*s + 0.5))

figure;nyquist(CG);grid on;

figure;bode(CG);grid on;hold all;

% let's create the sensitivity functon S(s) = 1/(1+C(s)G(s))
% feedback(A(s),B(s)) = A(s)/(1+A(s)B(s))
S = feedback(1,CG)
bode(S) % High peak sensitivity

% How to fix it via gain variation or notch filter
notch = (s^2 + 0.49)/(s^2 + 0.35*s + 0.49);

figure;
% bode of open loop systems 
subplot(3,1,1);bode(CG);grid on; hold on;
               bode(0.5*CG);   % with gain k = 0.5
               bode(notch*CG); % with notch filter
% bode of sensitivity
subplot(3,1,2);bode(feedback(1,CG));grid on; hold on;
               bode(feedback(1,0.5*CG));   % with gain k = 0.5
               bode(feedback(1,notch*CG)); % with notch filter
% step of closed loop systems with unity feedback
subplot(3,1,3);step(feedback(CG,1));grid on; hold on;
               step(feedback(0.5*CG,1));
               step(feedback(notch*CG,1));
