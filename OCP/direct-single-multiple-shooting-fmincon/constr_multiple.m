function [ cineq, ceq, Dcineq, Dceq ] = constr_multiple( Z )

global N R Q input x0 xref;

Nx = size(Q,1);
Nu = size(R,1);
Nz = Nx+Nu;

cineq = []; Dcineq = [];
ceq = [Z(1:Nx)-x0]; 
Dceq = zeros(N*Nz+Nx,N*Nx+2*Nx);
Dceq(1:Nx,1:Nx) = eye(Nx);
for k = 1:N
    x_k = Z((k-1)*Nz+1:(k-1)*Nz+Nx);
    u_k = Z((k-1)*Nz+Nx+1:k*Nz);
    x_next = Z(k*Nz+1:k*Nz+Nx);
    
    input.x = x_k;
    input.u = u_k;
    input.sens = 1;
    output = RK4_integrator( @ode, input );
    ceq = [ceq; output.value-x_next];
    Dceq((k-1)*Nz+1:k*Nz+Nx,Nx+(k-1)*Nx+1:Nx+k*Nx) = [output.sensX output.sensU -eye(Nx)].';
end

ceq = [ceq; Z(N*Nz+1:end)-xref];
Dceq(N*Nz+1:end,Nx+N*Nx+1:end) = eye(Nx);

end