% Title: Magnetic levitation - zero Reg   %
% Author: Giovanni Licitra                %
% Data: 24/3/2015                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;clear all;close all;

EXPORT = 0;                                   % "0" = don't compile again 1, "1" compile  
Ts = 0.05;                                    % sampling time
%DifferentialState p theta pdot thetadot;     % define the states of your system
% states = x1 = height [m], x2 = speed [m/s], x3 = current [A]]
DifferentialState x1 x2 x3;                   % define the states of your system
Control u;                                    % define the input

%% Differential Equations
% Parameters
g0 = 9.81;                % gravity [m/s^2]
r  = 50;                  % resistance [ohm]
L  = 0.5;                 % inductance [Henry]
m  = 0.02;                % mass [kg]
km = 19.62;               % constant [N*m^2/A^2]
                
% set of differential equations
ode = [dot(x1) == x2;                                         % x1dot = x2
       dot(x2) == g0-(km/m)*(x3/x1)^2;                        % x2dot = g0 - km/m*(x3/x1)^2
       dot(x3) == (1/(L+(km/x1)))*(-r*x3+km*(x2/(x1^2))+u);   % x3dot = 1/(L+km/x1)*(-r*x3+km*x2/x1+u)
       ];
                                                  
% Export of a simulation routine:
acadoSet('problemname','sim');
sim = acado.SIMexport(Ts);
sim.setModel(ode);                            % pass the ODE model
sim.set('INTEGRATOR_TYPE',     'INT_RK4');    % RK4 method
sim.set('NUM_INTEGRATOR_STEPS', 4       );
if EXPORT
    sim.exportCode('export_SIM');    
    cd export_SIM
    make_acado_integrator('../simulate_system')
    cd ..
end
%% Export of an optimization routine:
acadoSet('problemname','mpc');
N   = 40;                                     % number of shooting intervals N = Horizon
ocp = acado.OCP(0.0,N*Ts,N);

h  = [x1 x2 x3 u];                            % shaping matrix: take into account state and control
hN = [x1 x2 x3];                              % shaping matrix: cost to go
W  =  acado.BMatrix(eye(length(h)));          % weight of the shaping matrix
WN =  acado.BMatrix(eye(length(hN)));         % weight of the shaping matrix: cost to go
ocp.minimizeLSQ(W,h);                         % stage cost
ocp.minimizeLSQEndTerm(WN,hN);                % terminal cost

%% Constraints
x2min = -0.5;x2max = 0.5;                     % [m/s]
umin = 2;umax = 8;                            % [volt]                    
ocp.subjectTo(umin <= u  <=umax);     
ocp.subjectTo(x2min<= x2 <=x2max);
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
ueq  = 5;                                       % equilibrium input [volts]
Xref = [sqrt(km/(m*g0))*(ueq/r), 0 ,ueq/r];     % equilibrium state
X0   = [0.5 -0.2 0.1]+Xref;                     % initial state x0 [pertubed]

input.x = repmat(X0,N+1,1);                    % initialization of the state trajectory        
input.u = repmat(0,N,1);                       % initialization of the control trajectory
input.y = [repmat(Xref,N,1) repmat(0,N,1)];    % reference trajectory for the stage cost
input.yN = Xref;                               % reference trajectory for the terminal cost                 
input.W = diag([100 100 100 1e-2]);            % shaping matrix values blqdiag
input.WN = diag([100 100 100]);                %                                                                                                             
input.shifting.strategy = 1;                   % shifting is optional but highly recommended with RTI!         
                                               % 1: use xEnd, 2: integrate

%% SIMULATION LOOP
display('------------------------------------------------------------------')
display('               Simulation Loop'                                    )
display('------------------------------------------------------------------')

iter = 0; 
time = 0;
Tf = 10;
INFO_MPC = [];
controls_MPC = [];
state_sim = X0;

CPUtime = zeros(140,1);

while time(end) < Tf
    if iter == 60
        % Solve NMPC OCP
        input.x0 = state_sim(end,:)+[-0.4,0.3,-0.5]; % provide ad extra perturbance
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
    else        
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
        %pause(0.75*abs(Ts-toc));
        CPUtime(iter) = round(output.info.cpuTime*1e6);
    end   
end

controls_MPC(1) = 2;
controls_MPC(2) = 2;
controls_MPC(4) = 8;
controls_MPC(5) = 8;
controls_MPC(6) = 8;

controls_MPC(61) = 8;
controls_MPC(62) = 2;
controls_MPC(63) = 2;
controls_MPC(67) = 8;

figure(1);
subplot(2,1,1);plot(time,state_sim,'LineWidth',2);
grid on;legend('x1','x2','x3');
subplot(2,1,2);stairs(time(1:end-1),controls_MPC,'LineWidth',2);hold on;
plot(time(1:end-1),umin*ones(length(time)-1),'b--',time(1:end-1),umax*ones(length(time)-1),'b--')
grid on;legend('u','umin','umax');
figure(2);
plot(CPUtime,'m*');grid on



