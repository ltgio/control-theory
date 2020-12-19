function [ cost, Dcost ] = cost_single( Z )                 
global N R;

cost = 0;
Dcost = [];
for k = 1:N
    u_k = Z(:,k);
    cost = cost + u_k.'*R*u_k;       
    Dcost = [Dcost; R*u_k];
end
cost = 1/2*cost;

end