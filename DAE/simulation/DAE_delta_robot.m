clear all;
close all;
clc;

% Load CasADi
import casadi.*

%% [Task 2.1]: Modeling the delta robot
% Problem constants:
g = 9.81;
m = 1;
J = 1;
l = 0.2;
L = 0.6;

% Symbolic variables:
p  = SX.sym('p',3);
dp = SX.sym('dp',3);

alpha  = SX.sym('alpha',3);
dalpha = SX.sym('dalpha',3);

lam = SX.sym('lam',3);
u   = SX.sym('u',3);

% Potential energy:
V = m*g*p(3);

% Kinetic energy:
T = 0.5*m*dp.'*dp;
for k = 1:3
    T = T + 0.5*J*dalpha(k)^2;
end

% Set-up of the constraints:
gamma = [0 2*pi/3 4*pi/3];
c = SX.zeros(3,1);
dist = 2*l; % How far is the motors attachment point from the center
for k = 1:3
    R{k}     = [cos(gamma(k))  sin(gamma(k))  0 ;
               -sin(gamma(k))  cos(gamma(k))  0 ;
                    0               0         1 ];
                    
    p_arm0{k} = R{k}*[ dist;
                         0;
                         0  ];  % motors attachment point
                    
    p_arm{k} = R{k}*[ l*cos(alpha(k)) + dist;
                        0;
                       -l*sin(alpha(k))];
    
    c(k,1) = (p-p_arm{k}).'*(p-p_arm{k}) - L^2;
end
p_armF = Function('p_armF',{alpha},p_arm);

q  = [p;alpha];
dq = [dp;dalpha];

cFun = Function('cFun',{q},{c});

% The Lagrange function:
Lag = T - V - lam.'*c;

Fg = [zeros(3,1); u];  % u are the motor torques

Ldq = jacobian(Lag,dq);
Lq = jacobian(Lag,q);

% Ldq_dot = jacobian(Ldq,q)*dq + jacobian(Ldq,dq)*ddq;
% Is this DAE indeed semi-explicit?
M = full(DM(jacobian(Ldq,dq))); % mass matrix
ddq = inv(M)*(Lq.'+Fg-jacobian(Ldq,q)*dq); % ddq explicitly defined

% DAE model :  [  dot(q) == dq;
%                dot(dq) == ddq;
%                   0    == c    ]

%% [Task 2.2]: Index reduction
% Is it of index 0 or 1?
full(DM(jacobian(c,lam)))

% Index reduction:
dc = jacobian(c,q)*dq;
% Is this new DAE of index 1?
full(DM(jacobian(dc,lam)))

% Index reduction:
ddc = jacobian(dc,q)*dq + jacobian(dc,dq)*ddq;
% Is this new DAE of index 1?
jacobian(ddc,lam)

dcFun = Function('dcFun',{[q;dq]},{dc});
dcJ = Function('dcJ',{q},{jacobian(c,q)});

ssEq = [ddq(4:6);ddc];
ssFun = Function('ssFun',{[q;dq;lam],u},{ssEq});
ssJac = Function('ssJac',{[q;dq;lam],u},{jacobian(ssEq,[lam;u])});

%% [Task 2.3]: Find a consistent initial condition
q0 = [0;0;-0.5*L;pi/2;pi/2;pi/2];
% Is this value for q consistent with c?
cTest = cFun(q0);
cTest = full(cTest)

% Find a consistent point using Newton's method:
while norm(cTest) > 1e-10
    jac   = dcJ(q0);  
    jac   = full(jac);
    val   = cFun(q0); 
    cTest = full(val);
    q0    = q0 - jac\cTest;
end
dq0 = zeros(6,1);

% Find the corresponding steady state torques:
lam0 = zeros(3,1);
u0 = zeros(3,1);
ssTest = ssFun([q0;dq0;lam0],u0);
ssTest = full(ssTest)
while norm(ssTest) > 1e-10
    jac = ssJac([q0;dq0;lam0],u0); jac = full(jac);
    val = ssFun([q0;dq0;lam0],u0); ssTest = full(val);
    delta =  -jac\ssTest;
    
    lam0 = lam0 + delta(1:3);
    u0 = u0 + delta(4:6);
end

%% [Task 2.4]: Use CasADi to simulate the DAE:
% Define integrator 
h = 0.1;
opts = struct('tf',h,'abstol',1e-10,'reltol',1e-10);
% dae = struct('x',[q;dq],'z',lam,'p',u,'ode',[dq;ddq],'alg',ddc);
% Baumgarte stab
gamma1 = 1;
gamma2 = 2;
dae = struct('x',[q;dq],'z',lam,'p',u,'ode',[dq;ddq],'alg',ddc+gamma1*dc+gamma2*c);
F = integrator('F', 'idas', dae, opts);

% Simulation loop
Tf = 20;
xs = [q0;dq0];
cs = []; dcs = [];
curT = 0;
steady_state = 0;
while curT < Tf
    % control inputs to apply
    if steady_state
        u_cur = u0;
    else
        u_cur = u0.*[1;1+4*sin(pi*curT);1];
    end
    
    Fout = F('x0',xs(:,end),'p',u_cur);
    xs = [xs full(Fout.xf)];
    
    cTest = cFun(xs(1:6,end));
    cs = [cs norm(full(cTest))];
    
    dcTest = dcFun(xs(:,end));
    dcs = [dcs norm(full(dcTest))];
    
    % Visualize
    figure(1);
    clf
    subplot(2,3,[1 2 4 5]);
    p = xs(1:3,end);
    p_arm = p_armF(xs(4:6,end));
    for k = 1:3
        line([1.5*p_arm0{k}(1) 0],[1.5*p_arm0{k}(2) 0],[1.5*p_arm0{k}(3) 0],'color','k','linestyle','--');hold on
        line([p_arm0{k}(1) full(p_arm(1))],[p_arm0{k}(2) full(p_arm(2))],[p_arm0{k}(3) full(p_arm(3))],'color','k','linewidth',2);hold on
        line([p(1)        full(p_arm(1))],[p(2)        full(p_arm(2))],[p(3)        full(p_arm(3))],'color','b','linewidth',1.7);hold on
    end
    plot3(p(1),p(2),p(3),'marker','o','markersize',12,'color','r');hold on
%   axis equal
    view(50,15)
    xlabel('x'); ylabel('y'); zlabel('z');
    zlim([-0.7 0])
    title(['current time: ' num2str(curT) 's']);
    
    subplot(2,3,3);
    semilogy(h.*[1:length(cs)],cs);
    xlabel('time (s)')
    ylabel('C')
    
    subplot(2,3,6);
    semilogy(h.*[1:length(cs)],dcs);
    xlabel('time (s)')
    ylabel('Cdot')
    
    drawnow
    
    curT = curT + h;
end
