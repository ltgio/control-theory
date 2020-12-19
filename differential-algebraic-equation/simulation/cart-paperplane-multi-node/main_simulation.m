%% Plane and Cart with cable shape in DAE formulation [2D]
% Version: Matlab 2014a/ casadi v3.0.0. rc2
% Author:  Marie Curie PhD student Jonas Koenemann & Giovanni Licitra
% Data:    16-02-2016
% addpath('D:\Software\casadi-matlabR2014a-v3.0.0-rc2')

import casadi.*
% Parameters =============================================================
p                          = struct;

p.massPlane                = 20;
p.massCart                 = 4;
p.massCable                = 1;

p.AspectRatio              = 10;
p.AreaCart                 = 1^2;

p.airDensity               = 1.23; 
p.gravity                  = 9.81; 
p.wingArea                 = 0.5; 
p.aeroDragCoefficientPlane = 0.04;                                  % for StreamLine Body
p.aeroDragCoefficientCart  = 0.2;                                   % long cylinder      (static value)   Df = 0.82 


mPlane        = p.massPlane;
mCart         = p.massCart;
mCable        = p.massCable;

AR            = p.AspectRatio;
Acart         = p.AreaCart;

rho           = p.airDensity;
g             = p.gravity;
Sref          = p.wingArea;
cDplane       = p.aeroDragCoefficientPlane;                
cDaeroCart    = p.aeroDragCoefficientCart;                

% numer of segments/nodes
N = 3;            % number of nodes
lengthCable   = 50;
lengthSegment = lengthCable/N;          % length of segment

cableDiam     = 0.01;
cD_node = 0.82;       % tether drag coefficient 
mNode   = mCable/N;     % mass node 

simulationTime = 20;  % seconds
simulationTimestep = 0.02;

% array to collect variables on the way
variables = {};

alpha = SX.sym('alpha');

% cart
pCart = SX.sym('p_cart',1);
vCart = SX.sym('v_cart',1);
variables = [variables, {pCart, vCart}];

% forces on the cart 
FaeroDragCart  = -0.5*rho*vCart^2*cDaeroCart*Acart* vCart/abs(vCart);   % air resistance
FCart          =  FaeroDragCart;

FNodes = [];
pNodes = [];
vNodes = [];

% N cable mass nodes with acting drag and tension forces
for k=1:N
  pNode = SX.sym(['p_' num2str(k)],2);
  vNode = SX.sym(['v_' num2str(k)],2);
  variables = [variables, {pNode, vNode}];
  
  % forces on the cable mass nodes 
  V              = norm_2(vNode);
  Fdrag          = -0.5*V^2*cD_node*cableDiam*lengthSegment * vNode / V;
  Fgravity       = [0;mNode*g];
  
  FNode          = Fdrag+Fgravity;
  
  FNodes = [FNodes,FNode];
  pNodes = [pNodes,pNode];
  vNodes = [vNodes,vNode];
  
end

% aircraft is last node
pPlane        = pNodes(:,end);
vPlane        = vNodes(:,end);

CL             = 2*pi*alpha*(10/12);          % Lift Coefficient
CD             = cDplane+CL^2/(AR*pi);         % Drag Coefficient
V              = norm_2(vPlane);   % velocity

eL             = 1/V*[ vPlane(2);-vPlane(1)];     % Lift Direction
eD             = 1/V*[-vPlane(1);-vPlane(2)];     % Drag Coefficient

Flift          = 0.5*rho*V^2*CL*Sref*eL;      % Lift Force
Fdrag          = 0.5*rho*V^2*CD*Sref*eD;      % Drag Force
Fgravity       = [0;mPlane*g];                % Gravity Force
Faero          = Flift+Fdrag+Fgravity;        % Total Aircraft Force

FNodes(:,end) = Faero;
% --------------------------------
% add tension forces

tensionVariables = {};

% tension force on cart and first node
tension = SX.sym('t_cart');
tensionVariables = [tensionVariables, {tension}];
FtCart    = (pNodes(:,1) - [pCart;0]) / norm_2(pNodes(:,1) - pCart) * tension; 

FCart = FCart + FtCart(1);
FNodes(:,1) = FNodes(:,1)-FtCart;

% tension forces on nodes
for k=1:N-1
  tension = SX.sym(['t_' num2str(k)]);
  tensionVariables = [tensionVariables, {tension}];
  FtNode = (pNodes(:,k+1) - pNodes(:,k)) / norm_2(pNodes(:,k+1) - pNodes(:,k)) * tension;
  FNodes(:,k) = FNodes(:,k) + FtNode;
  FNodes(:,k+1) = FNodes(:,k+1) - FtNode;
end



% ode withoout constraint equations

aCart = FCart / mCart;
aNodes = FNodes ./ mNode;
aNodes(:,end) = FNodes(:,end) ./ mPlane;

% constraint equations

% between cart and first node
pNode = pNodes(:,1);
vNode = vNodes(:,1);
aNode = aNodes(:,1);



C   = norm_2([pCart;0]-pNode)^2 - lengthSegment^2;
dC  = jtimes(C,[pCart;pNode], [vCart;vNode]);
ddC = jtimes(dC,[pCart;pNode], [vCart;vNode]) + jtimes(dC,[vCart;vNode], [aCart;aNode]);

algebraic_equations = ddC;
constraints = C;
dConstraints = dC;

% between nodes
for k=1:N-1
  pPrevNode = pNodes(:,k);
  vPrevNode = vNodes(:,k);
  aPrevNode = aNodes(:,k);

  pNode = pNodes(:,k+1);
  vNode = vNodes(:,k+1);
  aNode = aNodes(:,k+1);

  C = norm_2(pPrevNode-pNode)^2 - lengthSegment^2;
  dC = jtimes(C,[pPrevNode;pNode], [vPrevNode;vNode]);
  ddC = jtimes(dC,[pPrevNode;pNode], [vPrevNode;vNode]) + jtimes(dC,[vPrevNode;vNode], [aPrevNode;aNode]);

  algebraic_equations = [algebraic_equations; ddC];
  constraints = [constraints; C];
  dConstraints = [dConstraints; dC];
end

% build up equation
equation = [vCart;aCart];
for k=1:N
  equation = [equation; vNodes(:,k); aNodes(:,k)];
end

% initialize integrator
dae = struct;
dae.x = vertcat(variables{:});
dae.z = vertcat(tensionVariables{:});
dae.p = alpha;
dae.ode = equation;
dae.alg = algebraic_equations;


%% Define integrator for DAE ==============================================
Ts     = 0.01;   % sample time
gamma1 = 1;
gamma2 = 2;
dae    = struct('x',dae.x,'z',dae.z,'p',dae.p,'ode',dae.ode,'alg',dae.alg); 

opts                           = struct;
opts.tf                        = Ts;    
opts.abstol                    = 1e-10;
opts.reltol                    = 1e-10;
modelIntegrator    = integrator('F', 'idas', dae, opts);                                  
%%==========================
%% SIMULATION
initialPlaneHeight = -lengthCable; % setup initial state
initialState       = [];

pCart0 = 0;
vCart0 = 15;

initialState = [pCart0; vCart0];

for k=1:N
  pNodek0 = [0; k * initialPlaneHeight / N];
  vNodek0 = [15; 0];
  initialState = [initialState; pNodek0; vNodek0];
end

initialAlgebraicVariables = zeros(N,1);

integratorParams = struct;
integratorParams.x0 = initialState;
integratorParams.p = -2 * pi/180;
integratorParams.z0 = initialAlgebraicVariables;

tsim = [];
Xsim = [];

tic
for i=1:simulationTime/simulationTimestep
  integrationStep = modelIntegrator('x0',integratorParams.x0,'p',integratorParams.p);
  integratorParams.x0 = integrationStep.xf;
  integratorParams.z0 = integrationStep.zf; 
  tsim = [tsim;i*Ts   ];   % time simulation
  Xsim = [Xsim;full(integrationStep.xf)'];   % Simulation states
end
toc

VisualizeMultiNodeSim(tsim,Xsim);

