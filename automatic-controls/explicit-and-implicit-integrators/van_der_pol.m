function [ dx ] = van_der_pol( t, x, mu)

% Van der Pol oscillator
dx = [mu*(x(1)-1/3*x(1).^3-x(2));
      1/mu*x(1)                ];

% Remark: the dynamics are time-invariant, we keep t as in input in order 
% to use this function with ode15s.

end

