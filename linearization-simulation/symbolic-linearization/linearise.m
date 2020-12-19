function [A,B,C,D] = linearise( plantModel )
%LINEARISE Summary of this function goes here
%   Detailed explanation goes here

J_A     = jacobian(plantModel.non_lin, plantModel.state);
J_B 	= jacobian(plantModel.non_lin, plantModel.input);

x_eq    = plantModel.EquilibriumState;
u_eq    = plantModel.EquilibriumInput; 

A = subs(J_A, plantModel.state, x_eq);
A = double(subs(A, plantModel.input, u_eq));

B = subs(J_B,plantModel.state, x_eq);
B = double(subs(B, plantModel.input, u_eq));

C = eye(size(A,1));       
D = zeros(size(A,1),size(B,2));  
    

