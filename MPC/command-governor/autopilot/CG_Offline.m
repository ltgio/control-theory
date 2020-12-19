function [A_Wd, b_Wd, Ax_Vx, AW_Vx, b_Vx] = CG_Offline(Phi,G,Hc,L,T,b,delta,k0)
% CG_Offline Calculate the matrices of the costraints for the optimum
% problem.
% Phi, G: matrices A,B of augmented system
%             x(t+1) = Phi*x + G*g
% Hc, L: matrices C,D of constrained variables
%               c(t) = Hc*x  + L*g
% T: projection matrix
% b: constraints vector
% k0: prediction horizon
% delta: algorithm tollerance

%% Preliminary

nx = size(Phi,1);   % number of states
nm = size(G,2);     % number of inputs
nz = size(T,1);     % number of constraints

%% STEADY STATE CONSTRAINTS            _____ 
% T(Hc*(I-PHI)'*G + L)w <= b - delta*\/Ti*Ti'
% \_________ _______/      \_______ ________/
%          A_Wd                   b_Wd

% Dimension A_Wd [nz x nr]
% Dimension b_Wd [nz x 1]
% Dimension   w  [nr x 1]

%                          _____ 
% Matrix of projection = \/Ti*Ti'
proj = zeros(nz,1);     % inizialize marix TixTi'
Tt   = T';              % transponse of matrix T

for m = 1:nz
  proj(m,1) = T(m,:)*Tt(:,m);
end

I = eye(nx);
invPhi = pinv(I-Phi);

% Building set W_delta
A_Wd = T*((Hc*invPhi*G) + L);
b_Wd = b - delta*sqrt(proj);

%% TRANSIENT CONSTRAINTS

% T(Hc*PHI^k*x + T*RkcW <= b 
% \____ ____/    \_ _/    \ /
%    Ax_Vx       AW_Vx    b_Vx

% Building set V(x), with k = 0 => Rk = 0
Ax_Vx     = T*Hc*(Phi^0);
AW_Vx     = T*L;
b_Vx      = repmat(b, (k0+1),1);

% Building of Rk
Rkc  = cell(1,k0);
Rk   = zeros(nx,nm);

for i=0:k0-1
  Rk = Rk + (Phi^i)*G;
  Rkc{i+1} = Hc*Rk + L;
end

% Complete set V(x), with k = 1,...,k0
for t = 1:k0
Ax_Vx   = [Ax_Vx; T*Hc*(Phi^t)];
AW_Vx   = [AW_Vx; T*(Rkc{t})];
end

end