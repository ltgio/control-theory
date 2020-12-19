% Title: Magnetic levitation - Set Point  %
% Author: Giovanni Licitra                %
% Data: 24/3/2015                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc;clear all;close all;

EXPORT = 0;                                   % "0" = don't compile again 1, "1" compile  
Ts = 0.05;                                    % sampling time

DifferentialState x1 x2 x3 x4 x5 x6;          % define the states of your system
Control Taero Faero;                          % define the input

%% Differential Equations
% Parameters
mass  = 3*10^4;           % massa veivolo [kg]
J     = 3*10^4;           % inerzia [kg*m^2]
l     = 4;                % lunghezza alare [m]
alpha = pi/8;             % angolo [rad]
g     = 9.81;             % accelerazione gravitazionale
                
% set of differential equations
ode = [dot(x1) == x4;                                         
       dot(x2) == x5;                        
       dot(x3) == x6; 
       dot(x4) == -Taero/mass*sin(x3) + 2*Faero/mass*sin(alpha)*cos(x3); 
       dot(x5) ==  Taero/mass*cos(x3) + 2*Faero/mass*sin(alpha)*sin(x3)-g;
       dot(x6) == 2*l*Faero/J*cos(alpha);
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
N = 80;                                       % number of shooting intervals N = Horizon
ocp = acado.OCP(0.0,N*Ts,N);

h  = [x1 x2 x3 x4 x5 x6 Taero Faero];         % shaping matrix: take into account state and control
hN = [x1 x2 x3 x4 x5 x6];                     % shaping matrix: cost to go
W  =  acado.BMatrix(eye(length(h)));          % weight of the shaping matrix
WN =  acado.BMatrix(eye(length(hN)));         % weight of the shaping matrix: cost to go
ocp.minimizeLSQ(W,h);                         % stage cost
ocp.minimizeLSQEndTerm(WN,hN);                % terminal cost

%% Constraints 
Fmin  = -2000; 
Fmax  =  2000;
Tmin  =  mass*g - 10000; 
Tmax  =  mass*g + 10000;

x4min = -1;
x4max =  1;

ocp.subjectTo(Tmin <= Taero <= Tmax);     
ocp.subjectTo(Fmin <= Faero <= Fmax);     
ocp.subjectTo(x4min<=  x4   <= x4max); % constraints in dy     

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
ueq = [mass*g , 0];                            % equilibrium input [volts]
xeq  = [0,0,0,0,0,0];                          % equilibrium state
X0   = [0.1,-0.1,0,0.5,0.3,0]+xeq;        % initial state
Xref = [2,2,0,0,0,0];                          % reference point

input.x = repmat(X0,N+1,1);                    % initialization of the state trajectory        
input.u = repmat(ueq,N,1);                     % initialization of the control trajectory
input.y = [repmat(Xref,N,1) repmat(ueq,N,1)];    % reference trajectory for the stage cost
input.yN = Xref;                               % reference trajectory for the terminal cost                 
input.W  = diag([1e3 1e3 1e2 1e-1 1e1 1e1 1e-6 1e-6]); % shaping matrix values blqdiag
input.WN = diag([1e3 1e3 1e2 1e-1 1e1 1e1]);       %                                                                                                             
input.shifting.strategy = 1;                   % shifting is optional but highly recommended with RTI!         
                                               % 1: use xEnd, 2: integrate

%% SIMULATION LOOP
display('------------------------------------------------------------------')
display('               Simulation Loop'                                    )
display('------------------------------------------------------------------')
 
time      = 0;
Tf        = 10;
state_sim = X0;

iter         = 0;
INFO_MPC     = [];
controls_MPC = [];
CPUtime      = [];

while time(end) < Tf
    tic
    if iter == 30
        % Solve NMPC OCP
        input.x0 = state_sim(end,:)+[0.4,-0.5,0,+0.5,0.4,-0.3];
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
        CPUtime   = [CPUtime;output.info.cpuTime*1e6];
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
        CPUtime   = [CPUtime;output.info.cpuTime*1e6];
    end
end

figure;
subplot(2,1,1);plot(time,state_sim(:,1:3),'LineWidth',2);hold on;
grid on;legend('y','z','\theta');
subplot(2,1,2);plot(time,state_sim(:,4:6),'LineWidth',2);hold on;
grid on;legend('dy','dz','d\theta');
plot(time,x4max*ones(length(time)),'r--');

figure;
subplot(2,1,1);stairs(time(1:end-1),controls_MPC(:,1),'LineWidth',2);
hold on;grid on;
plot(time(1:end-1),Tmin*ones(length(time)-1),'r--',time(1:end-1),...
                   Tmax*ones(length(time)-1),'r--')
subplot(2,1,2);stairs(time(1:end-1),controls_MPC(:,2),'LineWidth',2);
hold on;grid on;
plot(time(1:end-1),Fmin*ones(length(time)-1),'r--',time(1:end-1),...
                   Fmax*ones(length(time)-1),'r--')
figure;
plot(time(1:end-1),CPUtime,'m*');grid on
ylabel('microseconds');xlabel('time [s]')



