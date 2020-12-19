function [ n, m ] = getDimension( )

[~,b] = ode();
n = b(1);
m = b(2);

end

