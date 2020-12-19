function dae = CartPlaneModel(t,x,dx,u,p)
%
% State is 
%     positions of plane  [px;pz]
%     position of cart    pc
%     velocity of plane   [vx;vz]
%     velocity of cart    vc
%     tension force       T  which is an algebraic state
%
% Control is
%     angle of attack     alpha

%% Parameters =============================================================
mPlane        = p.massPlane;
AR            = p.AspectRatio;
rho           = p.airDensity;
g             = p.gravity;
Sref          = p.wingArea;
mCart         = p.massCart;
cD_plane      = p.aeroDragCoefficientPlane;                
cDaeroCart    = p.aeroDragCoefficientCart;                
cDrollCart    = p.rollDragCoefficientCart; 
Acart         = p.AreaCart;
%% States =================================================================
xPlane        = x(1);
zPlane        = x(2);
xCart         = x(3);
vxPlane       = x(4);
vzPlane       = x(5);
vxCart        = x(6);
lambda        = x(7);                         % algebraic state [tension]
%% Derivatives ============================================================
dxPlane       = dx(1);
dzPlane       = dx(2);
dxCart        = dx(3);
dvxPlane      = dx(4);
dvzPlane      = dx(5);
dvxCart       = dx(6);
%% Controls ===============================================================
alpha         = u;
%% Build Cable force ======================================================
l              = sqrt((xPlane - xCart)^2 + zPlane^2);      % cable length
Fcable         = [xPlane - xCart; zPlane] / l*lambda;      % direction * magnitude
Ft_cart        = Fcable(1);                                % tension force on cart    [Along x only]
Ft_plane       = Fcable;                                   % tension force on aircraft
%% Build Aircraft Model ===================================================
CL             = 2*pi*alpha*(10/12);          % Lift Coefficient       
CD             = cD_plane + CL^2/(AR*pi);     % Drag Coefficient
V              = norm([vxPlane;vzPlane],2);   % velocity

eL             = 1/V*[ vzPlane;-vxPlane];     % Lift Direction
eD             = 1/V*[-vxPlane;-vzPlane];     % Drag Coefficient

Flift          = 0.5*rho*V^2*CL*Sref*eL;      % Lift Force
Fdrag          = 0.5*rho*V^2*CD*Sref*eD;      % Drag Force
Fgravity       = [0;mPlane*g];                % Gravity Force
Faero          = Flift+Fdrag+Fgravity;        % Total Aircraft Force

ddxPlane       = (Faero - Ft_plane)/mPlane;     % total acceleration on aircraft

%% Build Cart Model =======================================================
FrollDragCart  = cDrollCart*mCart*g;
FaeroDragCart  = 0.5*rho*vxCart^2*cDaeroCart*Acart;
ddxCart        = (Ft_cart - FrollDragCart - FaeroDragCart) / mCart ;

% constraint equation determines the tension force T
% twice differentiation of 
% c = (xPlane-xCart)^2 + zPlane^2 - l0,                   l0 = cable length
%constraint_equation =   2*vzPlane^2 + 2*dvzPlane*zPlane + vxCart*(2*vxCart - 2*vxPlane)...
%                     - vxPlane*(2*vxCart - 2*vxPlane) + dvxCart*(2*xCart - 2*xPlane) - dvxPlane*(2*xCart - 2*xPlane);
constraint_equation = (vxPlane-vxCart)^2+(xPlane-xCart)*(dvxPlane-dvxCart)+...
                      (vzPlane- 0    )^2+(zPlane- 0   )*(dvzPlane - 0);
%% Build DAE Model (Implicit formulation) =================================
dae = [ dxPlane  - vxPlane    ; ...
        dzPlane  - vzPlane    ; ...
        dxCart   - vxCart     ; ...
        dvxPlane - ddxPlane(1); ...        % along x
        dvzPlane - ddxPlane(2); ...        % along y
        dvxCart  - ddxCart    ; ...
        constraint_equation  ];
end

