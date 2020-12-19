function Rosenbrock_casadi
%% Newton's method for optimization
%  Ex 3 summer TEMPO school 2015
%  KEYWORD
%  3D plot
%  Newton Method algorithm
%  HOWTO use fminunc function
%  HOWTO use fmincon function
%  Exact Newton's Method & Exact Newton Method

%% Part 0: Plotting Rosenbrock’s function
%plotRosenbrock
%% Part 1: Implementing Newton's method
z = [0; 0];                      % initial condition
res = 1; iter = 0;
while norm(res) > 1e-5
    [~,res,H] = D2f_eval(z);
    dz = -H\res;
    z = z + dz;        % x_k+1 = X_k - ivn(D2f)*Df
    iter = iter+1;
end
disp(['solution found in ' num2str(iter) ' iterations: x = ' num2str(z(1)) ', y = ' num2str(z(2))])

import casadi.*
x = SX.sym('x');
y = SX.sym('y');
f = 100*(y - x.^2).^2 + (1-x).^2;
z = [x;y];

D2f = Function('D2f_eval',{z},{f,jacobian(f,z)',hessian(f,z)});

iter = 0;
res  = 1;
zk   = [0;0];

while norm(res) > 1e-5
    [~,res,H] = D2f(zk);
    res = full(res);
    H   = full(H);
    dz = -H\res;
    zk = zk + dz;        % x_k+1 = X_k - ivn(D2f)*Df
    iter = iter+1;
end

disp(['solution found in ' num2str(iter) ' iterations: x = ' num2str(zk(1)) ', y = ' num2str(zk(2))])
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
function z = vrosenbrock(x,y)
  z = 100*(y - x.^2).^2 + (1-x).^2;
end
function plotRosenbrock
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
end