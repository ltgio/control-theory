function [ output ] = RK4_integrator( ode_fun, input )      
    x0 = input.x;
    u0 = input.u;
    Ts = input.Ts;
    nSteps = input.nSteps;
    
    nx = length(x0);
    nu = length(u0);
    h = Ts/nSteps;
    STEP = 1e-100;
    
    compute_sensitivities = ~isfield(input,'sens') || input.sens;
    
    xEnd = x0;
    A = eye(nx);
    B = zeros(nx,nu);
    for i = 1:nSteps
        x0 = xEnd;
        xEnd = rk4_step(ode_fun,x0,u0,h);
        if compute_sensitivities
            sensX = zeros(nx,nx); sensU = zeros(nx,nu);
            for j = 1:nx
                % imaginary trick for states
                xTemp1 = x0; xTemp1(j) = xTemp1(j) + STEP*sqrt(-1);
                xTemp1 = rk4_step(ode_fun,xTemp1,u0,h);
                
                sensX(:,j) = imag(xTemp1)./STEP;
            end
            for j = 1:nu
                % imaginary trick for controls
                uTemp1 = u0; uTemp1(j) = uTemp1(j) + STEP*sqrt(-1);
                xTemp1 = rk4_step(ode_fun,x0,uTemp1,h);
                
                sensU(:,j) = imag(xTemp1)./STEP;
            end
            % propagate sensitivities
            A = sensX*A;
            B = sensX*B + sensU;
        end
    end
    output.value = xEnd;
    if compute_sensitivities
        output.sensX = A;
        output.sensU = B;
    end
end
