clc;clear all;close all;

s = tf('s');
kp = 1;
ki = 2;
zd = 3;  % zero for derivative (derivative as lead compensator)
pd = 4;  % pole for derivative

%Cpid = (kp*s + ki + kd*s^2)/s improper tf
Cpid = ( kp*s*(s+pd) + ki*(s+pd) + s*(s+zd) )/( s*(s+pd) )
[num,den] = tfdata(Cpid);

num = cell2mat(num);
den = cell2mat(den);

[A,B,C,D] = tf2ss(num,den);

csys = ss(A ,B ,C ,D) % create the controllable canonical model
osys = ss(A',C',B',D) % create the observable canonical model
