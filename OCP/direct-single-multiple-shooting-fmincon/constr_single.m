function [ cineq, ceq, Dcineq, Dceq ] = constr_single( Z )  
global input N x0 xref;

cineq = [];  % inequality array 
ceq = [];    % equality array 
Dcineq = []; % gradient of inequality
Dceq = [];   % gradient of equality

x_k = x0;    % initial condition
for k = 1:N
    u_k = Z(:,k);
    input.x = x_k;
    input.u = u_k;
    output = RK4_integrator(@ode, input);
    x_k = output.value;
    if k > 1
        Dceq = [Dceq*output.sensX.'; output.sensU.'];
    else 
        Dceq = [Dceq; output.sensU.'];
    end    
end
ceq = [x_k - xref];

end