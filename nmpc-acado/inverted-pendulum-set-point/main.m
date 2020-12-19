%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Title: inverted pendulum with NMPC  %
% Author: Giovanni Licitra            %
% Data: 28/11/2014                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;clear all;close all;

EXPORT = 0;                                   % "0" = don't compile again 1, "1" compile  
Ts = 0.05;                                    % sampling time
DifferentialState p theta pdot thetadot;      % define the states of your system
Control F;                                    % define the input

%% Differential Equations
M = 1;m = 0.1;g = 9.81;l = 0.8;               % Parameters
ode = [dot(p)        == pdot;                 % set of differential equations
       dot(theta)    == thetadot;
       dot(pdot)     == (-m*l*sin(theta)*thetadot*thetadot-m*g*cos(theta)*sin(theta)+F)/(M+m-m*(cos(theta)*cos(theta)));
       dot(thetadot) == (-m*l*cos(theta)*sin(theta)*thetadot*thetadot+F*cos(theta)+(M+m)*g*sin(theta))/(l*(M+m-m*(cos(theta)*cos(theta))));
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

h  = [p theta pdot thetadot F];               % shaping matrix: take into account state and control
hN = [p theta pdot thetadot];                 % shaping matrix: cost to go
W  =  acado.BMatrix(eye(length(h)));          % weight of the shaping matrix
WN =  acado.BMatrix(eye(length(hN)));         % weight of the shaping matrix: cost to go
ocp.minimizeLSQ(W,h);                         % stage cost
ocp.minimizeLSQEndTerm(WN,hN);                % terminal cost

%% Constraints
xmin = -2;xmax = 2;                           % [m]
Fmin = -20;Fmax = 20;                         % [N]                    
ocp.subjectTo(Fmin<=F<=Fmax);     
ocp.subjectTo(xmin<=p<=xmax);
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
X0 =   [0 pi 0 0];                             % initial state (downward position)
Xref = [0 0 0 0];                              % reference point (upward position)
input.x = repmat(X0,N+1,1);                    % initialization of the state trajectory        
input.u = repmat(0,N,1);                       % initialization of the control trajectory
input.y = [repmat(Xref,N,1) repmat(0,N,1)];    % reference trajectory for the stage cost
input.yN = Xref;                             % reference trajectory for the terminal cost                    [p   0     0       0   ]
input.W = diag([1 1 1e-1 1e-1 1e-3]);          % shaping matrix values blqdiag[p=1 theta=1                 Q = [0 theta   0       0   ]  R = F
input.WN = diag([1 1 1e-1 1e-1]);              %                                                               [0   0    pdot     0   ]                                                   
input.shifting.strategy = 0;                   % shifting is optional but highly recommended with RTI!         [0   0     0   thetadot]
                                               % 1: use xEnd, 2: integrate

%% SIMULATION LOOP
display('------------------------------------------------------------------')
display('               Simulation Loop'                                    )
display('------------------------------------------------------------------')

iter = 0; time = 0;
Tf = 5;
INFO_MPC = [];
controls_MPC = [];
state_sim = X0;

while time(end) < Tf
    tic
    % Solve NMPC OCP
    input.x0 = state_sim(end,:);
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
    visualize(time, state_sim, Xref, xmin, xmax); 
    pause(0.75*abs(Ts-toc));
end

figure;
subplot(2,2,1);
plot(time,state_sim(:,1)); hold on;
plot([0 time(end)], [0 0], 'r:');
plot([0 time(end)], [xmin xmin], 'g--');
plot([0 time(end)], [xmax xmax], 'g--');
xlabel('time(s)');
ylabel('p');

subplot(2,2,2);
plot(time, state_sim(:,2)); hold on;
plot([0 time(end)], [0 0], 'r:');
xlabel('time(s)');
ylabel('theta');

subplot(2,2,[3 4]);
stairs(time(1:end-1), controls_MPC); hold on;
plot([0 time(end)], [0 0], 'r:');
plot([0 time(end)], [Fmin Fmin], 'g--');
plot([0 time(end)], [Fmax Fmax], 'g--');
xlabel('time(s)');
ylabel('F');



