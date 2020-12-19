function [ cost, Dcost ] = cost_multiple( Z )               

global N R Q;

Nx = size(Q,1);
Nu = size(R,1);
Nz = Nx+Nu;
cost = 0;
Dcost = [];
for k = 1:N
    x_k = Z((k-1)*Nz+1:(k-1)*Nz+Nx);         % take the state each time
    u_k = Z((k-1)*Nz+Nx+1:k*Nz);             % take the control each time
    cost = cost + x_k.'*Q*x_k + u_k.'*R*u_k; % define cost
    Dcost = [Dcost; Q*x_k; R*u_k];           % derivative of the cost
end
x_k = Z(N*Nz+1:end);                         % xN
cost = cost + x_k.'*Q*x_k;                   % add xN to the cost
Dcost = [Dcost; Q*x_k];                     
cost = 1/2*cost;

end