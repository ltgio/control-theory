function g = CG(r,x,Psi, A_Wd, b_Wd, Ax_Vx, AW_Vx, b_Vx)
%CG Calculate the reference signal of the Command Governor
%    w = CG(r,x,PSI,A_Wd, b_Wd, Ax_Vx, AW_Vx, b_Vx)
% where:
% r: original reference to track;
% x: current value of state vector;
%
% PSI: weight matrix for objective function;
%        OF = (w-r)'*PSI*(w-r)
%
% (A_Wd, b_Wd, Ax_Vx, AW_Vx, b_Vx): calculated by CG_Offline function.
%
% for Simulink: CG(u( : ), u( : ), Psi, A_Wd, b_Wd, Ax_Vx, AW_Vx, b_Vx)
%

opt= optimset('TolX',1e-09,'Display','Off') ;

CONSTRAINT_A = [A_Wd; AW_Vx];
CONSTRAINT_B = [b_Wd; b_Vx - Ax_Vx*x];


Obj_Fun = @(u)((u-r)'*Psi*(u-r));

g = fmincon(Obj_Fun, r, CONSTRAINT_A, CONSTRAINT_B,[],[],[],[],[],opt);      

end




