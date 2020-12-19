function [dx, dim] = ode( varargin )
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
nx = 4;
nm = 1;

if nargin == 0
dim = [nx nm];
dx = zeros(nx,1);
end

if nargin>0
    x = varargin{2};
    x1 = x(1);
    x2 = x(2); 
    x3 = x(3);
    x4 = x(4);
    u1 = varargin{3};
    p  = varargin{4};

    m    = p(1);
    rho  = p(2);
    g    = p(3);
    Sref = p(4);
    CL0  = p(5);
    CLa  = p(6);
    CD0  = p(7);
    CDa  = p(8);
    CDa2 = p(9);

    speed = [x3; x4];

    CL = CL0 + CLa*u1;
    CD = CD0 + CDa*u1 + CDa2*(u1^2);  
    Fx = 0.5*rho*norm(speed,2)*Sref*(x4*CL - x3*CD);  
    Fz = -0.5*rho*norm(speed,2)*Sref*(x3*CL + x4*CD) + m*g; 

    dx = [x3; ...
          x4; ...
          Fx/m; ...
          Fz/m];

end

end

