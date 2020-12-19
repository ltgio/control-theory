%% Chaos Synchronization - NMPC
% Giovanni Licitra

clc;clear all;close all;

EXPORT = 0;                                   % "0" = don't compile again 1, "1" compile  
Ts = 0.02;                                    % sampling time
DifferentialState x1 y1 z1 x2 y2 z2;          % define the states of your system
Control ux uy uz;                             % define the input
Nu = 3;                                       % define number of input 
%% Differential Equations
p = 10;q = 100/7;                             % Parameters
    
ode = [dot(x1) == p*(y1-(2*x1^3 - x1/7));     % set of differential equations
       dot(y1) == x1-y1+z1;
       dot(z1) == -q*y1;
       dot(x2) == p*(y2 - (2*x2^3 - x2/7)) + ux;
       dot(y2) == x2 - y2 + z2 + uy;
       dot(z2) == q*y2 + uz; 
      ]; 
%% Export of a simulation routine:
acadoSet('problemname','sim');
sim = acado.SIMexport( Ts );
sim.setModel(ode);                            % pass the ODE model
sim.set('INTEGRATOR_TYPE',     'INT_RK4');    % RK4 method
sim.set('NUM_INTEGRATOR_STEPS', 4       );
if EXPORT
    sim.exportCode( 'export_SIM' );    
    cd export_SIM
    make_acado_integrator('../simulate_system')
    cd ..
end
%% Export of an optimization routine:
acadoSet('problemname','mpc');
N = 40;                                       % number of shooting intervals N = Horizon
ocp = acado.OCP(0.0,N*Ts,N);

h  = [x1 y1 z1 x2 y2 z2 ux uy uz];            % shaping matrix: take into account state and control
hN = [x1 y1 z1 x2 y2 z2];                     % shaping matrix: cost to go
W  =  acado.BMatrix(eye(length(h)));          % weight of the shaping matrix
WN =  acado.BMatrix(eye(length(hN)));         % weight of the shaping matrix: cost to go
ocp.minimizeLSQ(W,h);                         % stage cost
ocp.minimizeLSQEndTerm(WN,hN);                % terminal cost

%% Constraints
%xmin = -0.04;xmax = 0.04;                    % 
umin = -1.5;umax = 1.5;                       %                     
ocp.subjectTo(umin <= ux <= umax);
ocp.subjectTo(umin <= uy <= umax);
ocp.subjectTo(umin <= uz <= umax);
%ocp.subjectTo(xmin<= y1 <=xmax);
ocp.setModel(ode);                            % pass the ODE model
mpc = acado.OCPexport(ocp);
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_RK4'           );  % RK4 method
mpc.set( 'NUM_INTEGRATOR_STEPS',        2*N                 );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
mpc.set( 'GENERATE_SIMULINK_INTERFACE', 'YES'           	);
if EXPORT
    mpc.exportCode( 'export_MPC' );    
    global ACADO_;
    copyfile([ACADO_.pwd '/../../external_packages/qpoases'], 'export_MPC/qpoases')    
    cd export_MPC
    make_acado_solver('../acado_MPCstep')
    cd ..
end

%% PARAMETERS SIMULATION
X0 = [0.02;0.05;-0.04;0.6;-0.5;-0.0004]';      % initial state 

Xref = [0 0 0 0 0 0];                          % reference point 
input.x = repmat(X0,N+1,1);                    % initialization of the state trajectory        
input.u = repmat(0,N,Nu);                      % initialization of the control trajectory
input.y = [repmat(Xref,N,1) repmat(0,N,Nu)];   % reference trajectory for the stage cost
input.yN = Xref.';                             % reference trajectory for the terminal cost 

C = [1 0 0 -1  0  0;
     0 1 0  0 -1  0;
     0 0 1  0  0 -1]; 
 
Q = 1000*C'*C;R = 1e-5*eye(3);
input.W = blkdiag(Q,R);
input.WN = Q;

%input.W = diag([1 1 1e-1 1e-1 1e-3]);         % shaping matrix values              
%input.WN = diag([1 1 1e-1 1e-1]);             %                                                                                                            

input.shifting.strategy = 1;                   % shifting is optional but highly recommended with RTI!         
                                               % 1: use xEnd, 2: integrate

%% SIMULATION LOOP
display('------------------------------------------------------------------')
display('               Simulation Loop'                                    )
display('------------------------------------------------------------------')

iter = 0; time = 0;
Tf = 40;
INFO_MPC = [];
controls_MPC = [];
state_sim = X0;

while time(end) < Tf
    tic
    % Solve NMPC OCP
    input.x0 = state_sim(end,:).';
    output = acado_MPCstep(input);
    % Save the MPC step
    INFO_MPC = [INFO_MPC; output.info];
    controls_MPC = [controls_MPC; output.u(1,:)];
    input.x = output.x;
    input.u = output.u; 
    % Simulate system
    sim_input.x = state_sim(end,:).';
    sim_input.u = output.u(1,:).';
    states = simulate_system(sim_input);
    state_sim = [state_sim; states.value'];
    iter = iter+1;
    nextTime = iter*Ts; 
    disp(['current time:' num2str(nextTime) '   ' char(9) ' (RTI step -- QP error: ' num2str(output.info.status) ',' ' ' char(2) ' KKT val: ' num2str(output.info.kktValue,'%1.2e') ',' ' ' char(2) ' CPU time: ' num2str(round(output.info.cpuTime*1e6)) ' µs)'])
    time = [time nextTime];     
end


visualizeChua(time',state_sim',controls_MPC')
