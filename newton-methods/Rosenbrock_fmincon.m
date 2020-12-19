function Rosenbrock_Newton_Method_fmincon
%% Newton's method for optimization
%  Ex 3 summer TEMPO school 2015
%  KEYWORD
%  3D plot
%  Newton Method algorithm
%  HOWTO use fminunc function
%  HOWTO use fmincon function
%  Exact Newton's Method & Exact Newton Method
%% Part 0: Plotting Rosenbrock’s function
% Create figure
figure1 = figure('Position',[1 400 1200 600]);
colormap('gray');
axis square;
R=0:.002:sqrt(2);
TH=2*pi*(0:.002:1); 
X = R'*cos(TH); 
Y = R'*sin(TH); 
Z = log(1+vrosenbrock(X,Y));

% Create subplot
subplot1 = subplot(1,2,1,'Parent',figure1);
view([124 34]);
grid('on');
hold('all');

surf(X,Y,Z,'Parent',subplot1,'LineStyle','none'); % Create surface
contour(X,Y,Z,'Parent',subplot1);                 % Create contour

% Create subplot
subplot2 = subplot(1,2,2,'Parent',figure1);
view([234 34]);
grid('on');
hold('all');

% Create surface
surf(X,Y,Z,'Parent',subplot2,'LineStyle','none');

% Create contour
contour(X,Y,Z,'Parent',subplot2);

%% Part 2: Solving the problem using fminunc
% fminunc's solution:
z0      = [0 0];
options = optimoptions(@fminunc,'Display','iter','Algorithm','quasi-newton');
x_sol   = fminunc(@f_eval,z0,options)

% fminunc's solution with gradient and hessian: (alternative method)
options = optimoptions(@fminunc,'Display','iter','Algorithm','trust-region','GradObj','on','Hessian','on');
x_sol1 = fminunc(@D2f_eval,z0,options)


%% Part 2: Solving the constrained problem using fmincon
% specify two function:
%    Df_eval for the cost
%    c_eval for the constraints

% OSS: the iteration of a Quasi-Newton Method are generally cheaper than
% Exact Newton scheme
options = optimoptions(@fmincon,'Display','iter', ...
    'Algorithm','interior-point','GradObj','on','GradConstr','on','Hessian','bfgs');
x_sol2 = fmincon(@Df_eval,z0,[],[],[],[],[],[],@c_eval,options)

% Exact Newton method: they are needed less iteration for converge to the
% solution 
% the exact Hessian is define in the function hessian_fun
options = optimoptions(@fmincon,'Display','iter', ...
    'Algorithm','interior-point','GradObj','on','GradConstr','on','Hessian','user-supplied','HessFcn',@hessian_fun);
x_sol3 = fmincon(@Df_eval,z0,[],[],[],[],[],[],@c_eval,options)

end

function f = f_eval(z)                   
x = z(1);y = z(2);
f = 100*(y - x.^2).^2 + (1-x).^2;            % function
end
function [f,df] = Df_eval(z)             
x = z(1);y = z(2);
f = 100*(y - x.^2).^2 + (1-x).^2;            % function
df = [2*(x-1)-400*x*(y-x.^2); 200*(y-x.^2)]; % gradient
end
function [f,df,ddf] = D2f_eval(z)        
x   = z(1);y = z(2);
f   = 100*(y - x.^2).^2 + (1-x).^2;                 % function
df  = [2*(x-1)-400*x*(y-x.^2); 200*(y-x.^2)];       % gradient
ddf = [2-400*(y-x.^2)+800*x.^2 -400*x; -400*x 200]; % hessian
end

function [c, ceq, dc, dceq] = c_eval(x)  
	% constraint function
  % constraint x1^2 + x2^2 < 1
    c  = x(1)^2 + x(2)^2 - 1;  % Inequality constraint array
	ceq  = [ ];                  % equality constraint array
    dc = [2*x(1); 2*x(2)];     % gradient of inequality
	dceq = [ ];                  % gradien of equality
end

function [ H ] = hessian_fun( z,lambda ) 
    % information about Hessian of the Lagrangian
    % D2L = D2f + lambda'*D2c where D2c = Hessian of inequality
    [~,~,ddf] = D2f_eval(z);
    H_c = [2 0; 0 2];
    H = ddf + lambda.ineqnonlin*H_c;
end

function z = vrosenbrock(x,y)
  z = 100*(y - x.^2).^2 + (1-x).^2;
end