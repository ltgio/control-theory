function [A,B,C,D] = getLinearise( plantModel )
%getLinearise returns A,B,C and D matrices by lieanrizing using jacobian
%function
%   Inputs require: model, data type=struct

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
    

