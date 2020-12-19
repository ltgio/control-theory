function simple_EEM
disp('LLS via state space formulation')
disp('the ode that generate the system is xdot = a*x + b u');
disp('where a = -0.5 b = 1')

N  = 2000;         % number of measurements
h  = 0.01;        % sampling time
u  = rand(N-1,1); % generate input
x0 = 0.2;         % set initial condition

%% generate data 
x_sim    = zeros(N,1);  % pre-allocation
x_sim(1) = x0;          % inital condition

for i=1:N-1 % simulate the system
  x_sim(i+1) = rk4_step(@ode,x_sim(i),u(i),h);
end

sigma_x = 0.001;
xn      = sigma_x*randn(N,1);
x       = x_sim + xn; % add some noise to the output 

%% LLS formulation
% theta* = (PHI'*PHI)^-1*PHI'*y_hat;

y_hat = x(2:end) - x(1:end-1);
% regressor matrix
PHI   = [h.*x(1:end-1), h.*u];

theta_truth = [-0.5;1]'
% compute fitting
theta_estimated   = [pinv(PHI)*y_hat]'

end

function x_next = rk4_step(ode_fun,x,u,h)                   
    k1 = ode_fun(x,u);
    k2 = ode_fun(x+h/2.*k1,u);
    k3 = ode_fun(x+h/2.*k2,u);
    k4 = ode_fun(x+h.*k3,u);
    x_next = x + h/6.*(k1+2*k2+2*k3+k4);
end

function dx = ode(x,u)
  a  = -0.5;  
  b  =  1  ;
  dx = a*x + b*u;
end