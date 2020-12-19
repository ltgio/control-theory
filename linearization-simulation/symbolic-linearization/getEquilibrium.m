function x_eq = getEquilibrium( plantModel )

% Find equilibriumm state through fsolve function
x = plantModel.state;
x_eq = fsolve(@(x) getModel(x, plantModel.EquilibriumInput, plantModel.p),...
    plantModel.InitialCondition );
x_eq = x_eq';

end

