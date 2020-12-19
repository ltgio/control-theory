function model = getModel( x, u, p )
%getModel Get the plant model from ode function
%   Form: getModel( x, u, p )
%   Inputs require: x -> states vector (symbolic form)
%                   u -> inputs vector (symbolic form)
%                   p -> parameters vector

    model = ode(0,x,u,p);

end

