%% Plane and Cart Model using DAE formulation [2D]
% Version: Matlab 2014a
% Author:  Marie Curie PhD student koenemann & Giovanni Licitra
% Data:    11-02-2016

clc;clear;close all;
import casadi.*
%% Paramters ==============================================================
mPlane       = 2;     % massPlane
AR           = 10;    % aspect Ratio
rho          = 1.23;  % airDensity
g            = 9.81;  % gravity
Sref         = 0.5;   % wingArea
mCart        = 1;     % massCart
cD_plane     = 0.04;  % aeroDragCoefficientPlane for StreamLine Body
cD_aeroCart  = 0.6;   % aeroDragCoefficientCart  long cylinder      (static value)   Df = 0.82
cD_rollCart  = 0.06;  % rollDragCoefficientCart  Tire - wet Asphalt (dynamic value) Df = 0.6  
Acart        = 0.1^2; % AreaCart

L            = 50;    % lengthCable 

%% Declare variables ======================================================
xPlane  = SX.sym('xPlane' ,1);
zPlane  = SX.sym('zPlane' ,1);
xCart   = SX.sym('xCart'  ,1);
vxPlane = SX.sym('vxPlane',1);
vzPlane = SX.sym('vzPlane',1);
vxCart  = SX.sym('vxCart' ,1);
alpha   = SX.sym('alpha'  ,1);  % input              [angle of attack]        
lambda  = SX.sym('lambda' ,1);  % algebraic variable [tension]

x  = [xPlane;zPlane;xCart;vxPlane;vzPlane;vxCart];
z  = lambda;
u  = alpha;

[nx,~] = size(x);
[nz,~] = size(z);
[nu,~] = size(u);

%% Build Cable force ======================================================
l        = sqrt((xPlane - xCart)^2 + zPlane^2); % cable length
Fcable   = [xPlane - xCart; zPlane] / l*lambda; % direction * magnitude
Ft_cart  = Fcable(1);                           % tension force on cart    [Along x only]
Ft_plane = Fcable;                              % tension force on aircraft

%% Build Aircraft Model ===================================================
CL       = 2*pi*alpha*(10/12);                  % Lift Coefficient       
CD       = cD_plane + CL^2/(AR*pi);             % Drag Coefficient
V        = norm_2([vxPlane;vzPlane]);           % velocity

eL       = 1/V*[ vzPlane;-vxPlane];             % Lift Direction
eD       = 1/V*[-vxPlane;-vzPlane];             % Drag Coefficient

Flift    = 0.5*rho*V^2*CL*Sref*eL;              % Lift Force
Fdrag    = 0.5*rho*V^2*CD*Sref*eD;              % Drag Force
Fgravity = [0;mPlane*g];                        % Gravity Force
Faero    = Flift+Fdrag+Fgravity;                % Total Aircraft Force

axPlane  = (Faero(1) - Ft_plane(1))/mPlane;     % total acceleration on aircraft
azPlane  = (Faero(2) - Ft_plane(2))/mPlane;     % total acceleration on aircraft

%% Build Cart Model =======================================================
FrollDragCart  = cD_rollCart*mCart*g;
FaeroDragCart  = 0.5*rho*vxCart^2*cD_aeroCart*Acart;
axCart         = (Ft_cart - FrollDragCart - FaeroDragCart)/mCart ;

xdot = [vxPlane;
        vzPlane;
        vxCart ;
        axPlane;
        azPlane;
        axCart];

%% Algebraic constraint ===================================================
q     = [xPlane;zPlane;xCart];
dq    = [vxPlane;vzPlane;vxCart];
ddq   = [axPlane;azPlane;axCart];

c     = ((xPlane - xCart)^2 + zPlane^2)-L^2 ;  
cFun  = Function('cFun',{q},{c}); 
full(DM(jacobian(c,lambda)))     % Is it of index 0 or 1?

dc    = jacobian(c,q)*dq;
dcFun = Function('dcFun',{[q;dq]},{dc});
dcJ   = Function('dcJ',{q},{jacobian(c,q)});
full(DM(jacobian(dc,lambda)))    % Is this new DAE of index 1?

ddc   = jacobian(dc,q)*dq + jacobian(dc,dq)*ddq;
jacobian(ddc,lambda)             % Is this new DAE of index 1?

%% Define integrator for DAE ==============================================
Ts     = 0.01;   % sample time
gamma1 = 1;
gamma2 = 2;
dae    = struct('x',x,'z',z,'p',u,'ode',xdot,'alg',ddc+gamma1*dc+gamma2*c); % with Baumgarte stabilization

opts                           = struct;
opts.tf                        = Ts;    
opts.abstol                    = 1e-10;
opts.reltol                    = 1e-10;
% opts.linear_solver             = 'csparse';
% opts.linear_solver_type        = 'user_defined';
% opts.steps_per_checkpoint      = 1;
% opts.exact_jacobian            = true;
% opts.calc_ic                   = true;
% opts.linear_solver_type        = 'iterative';
% opts.implicit_solver           = 'newton';
% opts.number_of_finite_elements = 1;
% opts.number_of_finite_elements = 1;
% opts.jit                       = true;
% opts.compiler                  = 'gcc';
%opts.max_num_steps             = 3000;
F    = integrator('F', 'idas', dae, opts);                                  

%% Simulation 
N    = 1000;         % N step
tsim = [0:N].*Ts;   % array time 
Xsim = zeros(N,nx); % state array
Zsim = zeros(N,nz); % algebraic array
cs   = zeros(N,nz); % c  array
dcs  = zeros(N,nz); % dc array 
ddcs = zeros(N,nz); % ddc array

x0        = [0; -50; 0; 13.0831; 0; 10];
Xsim(1,:) = x0;

u0   = 4*pi/180;

for i = 1:N
    Fout        = F('x0',Xsim(i,:),'p',u0);
    Xsim(i+1,:) = full(Fout.xf);
    ddcs(i+1,:) = full(Fout.zf);

    dc_out      = dcFun(Xsim(i,1:6));
    dcs(i+1,:)  = full(dc_out);
    
    c_out     = cFun(Xsim(i,1:3));
    cs(i+1,:) = full(c_out);  
end

VisualizePlaneCart_2014b(tsim,Xsim,cs,dcs,ddcs)