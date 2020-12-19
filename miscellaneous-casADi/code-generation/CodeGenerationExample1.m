clc;clear all;
import casadi.*

%% Generated C code can be as simple as calling the generate member function for a Function instance.
x = MX.sym('x',1);
f = Function('f',{x},{x^2},{'x'},{'r'});
opts = struct('main',true,'mex',true);
%opts = struct('mex',true);
f.generate('gen1.c',opts);
mex gen1.c -largeArrayDims

%%
r = gen1('f',5)
r = full(r)

%% generate a C file containing multiple CasADi functions by working with CasADi's CodeGenerator class:
f = Function('f',{x,y},{sin(y)*x},{'x','y'},{'q1'});
g = Function('g',{x,y},{cos(y)*x},{'x','y'},{'q2'});
opts = struct('mex',true);
C = CodeGenerator('gen2.c',opts);
C.add(f);
C.add(g);
C.generate();
mex gen2.c -largeArrayDims

%% execute the function from the command line and MATLAB
f_out = gen2('f',1,2)
g_out = gen2('g',1,2)
