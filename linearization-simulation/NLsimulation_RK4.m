function main_NLsimulation_and_LQR
%% Non linear simulation and the Linear-Quadratic regulator
%  Ex1: summer TEMPO school 2015
%  KEYWORD
%  Non Linear Simulation
%  RK4_integrator
%  Sensitivity [Ad,Bd]
%  Save and Load mat file
%  Animation

x0 = [0;0;0;0];u = 0.01; % initial condition [test no feedback]
T = 4; N = 80; Ts = T/N; % info time
%% Nonlinear simulation using the RK4 integrator
% input for RK4_integrator
x_rk4 = x0;
input.Ts = Ts;
input.nSteps = 2;
input.u = u;

t_rk4 = [0:N].*Ts;  % array time 

for i = 1:N
    input.x = x_rk4(:,end);
    output = RK4_integrator( @ode, input );
    x_rk4(:,end+1) = output.value;
    %visualize(t_rk4, x_rk4, 0.8);
    %pause(Ts/2)
end

% Comparison with ode45
[t_ode45,x_ode45] = ode45(@(t,x) ode(t,x,u), [0 T], x0);

%% Plot results [no feedback]
plot_result(t_ode45,t_rk4,x_ode45,x_rk4)

%% Discretize and linearize the dynamic system:
x_lin = zeros(4,1); u_lin = 0;         % linearization point
input.x = x_lin; input.u = u_lin;      
output = RK4_integrator( @ode, input );
A = output.sensX;                      % get Ad
B = output.sensU;                      % get Bd

%% Design of the LQR controller using dlqr:
Q = diag([1 1 1e-1 1e-1]); R = 1e-1;
[K,P] = dlqr(A,B,Q,R);

% save lqr.mat A B Q R K P
% load lqr.mat A B Q R K P

%% Close Loop simulation
l = 0.8;
x0 = [0 0.1 0 0].';
Ts = 0.05;
input.Ts = Ts;
input.nSteps = 3;

time = 0;
Tf = 5;
state_sim = x0;
state_lin = x0;
iter = 0;
    while time(end) < Tf
        % optimal feedback law (open loop)
        % u_LQR = -K*state_lin(:,end);      % open loop simulation
        u_LQR = -K*state_sim(:,end);        % close loop simulation

        % apply control to nonlinear system
        input.x = state_sim(:,end);
        input.u = u_LQR;
        output = RK4_integrator( @ode, input );
        state_sim(:,end+1) = output.value;

        % linear system
        state_lin(:,end+1) = A*state_lin(:,end) + B*u_LQR;

        % next time step and visualize result
        iter = iter+1;
        time(end+1) = iter*Ts;
        visualize(time, state_sim, l);

        pause(Ts/2);
    end
end


function dx = ode(t,x,u)         % inverted Pendulum  

    p = x(1); theta = x(2); v = x(3); omega = x(4);
    F = u(1);
    
    M = 1;
    m = 0.1;
    g = 9.81;
    l = 0.8;

    dx = [   v; ...
             omega; ...
             (-l*m*sin(theta)*omega^2 + F + g*m*cos(theta)*sin(theta))/(M + m - m*cos(theta)^2); ...
             (-l*m*cos(theta)*sin(theta)*omega^2 + F*cos(theta) + g*(m+M)*sin(theta))/(l*(M + m - m*cos(theta)^2)) ];

end
function [ output ] = RK4_integrator( ode_fun, input )
    % output: 
    %   Value: output
    %   SensX: matrix Ad [discrete time]
    %   SensU: matrix Bd [discrite time]
    
    % input
    x0 = input.x;               % current state         
    u0 = input.u;               % current input
    Ts = input.Ts;              % sample time
    nSteps = input.nSteps;      % n of steps
    
    nx = length(x0);
    nu = length(u0);
    h = Ts/nSteps;
    STEP = 1e-100;
    
    compute_sensitivities = ~isfield(input,'sens') || input.sens;
    
    xEnd = x0;
    A = eye(nx);
    B = zeros(nx,nu);
    for i = 1:nSteps
        x0 = xEnd;
        xEnd = rk4_step(ode_fun,x0,u0,h);
        if compute_sensitivities
            sensX = zeros(nx,nx); sensU = zeros(nx,nu);
            for j = 1:nx
                % imaginary trick for states
                xTemp1 = x0; xTemp1(j) = xTemp1(j) + STEP*sqrt(-1);
                xTemp1 = rk4_step(ode_fun,xTemp1,u0,h);
                
                sensX(:,j) = imag(xTemp1)./STEP;
            end
            for j = 1:nu
                % imaginary trick for controls
                uTemp1 = u0; uTemp1(j) = uTemp1(j) + STEP*sqrt(-1);
                xTemp1 = rk4_step(ode_fun,x0,uTemp1,h);
                
                sensU(:,j) = imag(xTemp1)./STEP;
            end
            % propagate sensitivities
            A = sensX*A;
            B = sensX*B + sensU;
        end
    end
    output.value = xEnd;
    if compute_sensitivities
        output.sensX = A;
        output.sensU = B;
    end
end
function x_next = rk4_step(ode_fun,x,u,h)             
    k1 = ode_fun(0,x,u);
    k2 = ode_fun(0,x+h/2.*k1,u);
    k3 = ode_fun(0,x+h/2.*k2,u);
    k4 = ode_fun(0,x+h.*k3,u);
    x_next = x + h/6.*(k1+2*k2+2*k3+k4);
end
function [] = visualize(time, state_sim, l)           

clf
whitebg([1.0 1.0 1.0])
set(gcf,'Color',[1 1 1])
hold on;

text(0.6,1.2,['current time: ' num2str(time(end)) 's'],'FontSize',15);
plot(state_sim(1,end),0,'ks','MarkerSize',30,'Linewidth',3);

theta = state_sim(2,end);
xB = state_sim(1,end)-l*sin(theta);
yB = l*cos(theta);

line([state_sim(1,end) xB], [0 yB], 'Linewidth',2);
plot(xB,yB,'ro','MarkerSize',25,'Linewidth',3);

grid on;
xlim([-1.5 1.5]);
ylim([-1.5 1.5]);
end
function [] = plot_result(t_ode45,t_rk4,x_ode45,x_rk4)
    figure;
    set(gcf,'Color',[1 1 1])
    subplot(221);
    plot(t_ode45,x_ode45(:,1),'--g'); hold on
    plot(t_rk4,x_rk4(1,:),'rx','MarkerSize',10);
    xlabel('time(s)')
    ylabel('p')
    legend('ode45', 'RK4');
    grid on;

    subplot(222);
    plot(t_ode45,x_ode45(:,2),'--g'); hold on
    plot(t_rk4,x_rk4(2,:),'rx','MarkerSize',10);
    xlabel('time(s)')
    ylabel('\theta')
    legend('ode45', 'RK4');
    grid on;

    subplot(223);
    plot(t_ode45,x_ode45(:,3),'--g'); hold on
    plot(t_rk4,x_rk4(3,:),'rx','MarkerSize',10);
    xlabel('time(s)')
    ylabel('v')
    legend('ode45', 'RK4');
    grid on;

    subplot(224);
    plot(t_ode45,x_ode45(:,4),'--g'); hold on
    plot(t_rk4,x_rk4(4,:),'rx','MarkerSize',10);
    xlabel('time(s)')
    ylabel('\omega')
    legend('ode45', 'RK4');
    grid on;
end