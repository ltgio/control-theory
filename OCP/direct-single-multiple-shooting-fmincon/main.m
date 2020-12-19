%% Direct single and multiple shooting
%  Ex4 - summer TEMPO school

%% Part 0: Define parameters inverted pendulum swing-up
global N Q R x0 xref; 

Nx = 4;     % number of differential states
Nu = 1;     % number of control inputs
Nz = Nx+Nu; 
N = 30;     % number of shooting intervals
T = 1.5;    % Horizon length
Ts = T/N;
x0 = [0; pi; 0; 0];    % initial condition
xref = [0; 0; 0; 0];   % final condition
Q = diag([0 0 0 0]);   
R = 1;
umax = 20;

global input;
input.Ts = Ts;
input.nSteps = 1;
input.sens = 1;

%% Part 1: Single shooting method

% Bound values:
LB_SS = -umax*ones(N*Nu,1);
UB_SS = umax*ones(N*Nu,1);

% Initialize single shooting variables:
Z_SS = zeros(Nu,N);

% Setting options and solving the problem with fmincon + SINGLE SHOOTING:
options = optimoptions(@fmincon,'Display','iter','Algorithm','interior-point','GradObj','on','GradConstr','on','Hessian','user-supplied','HessFcn',@hessian_single);
sol_single = fmincon(@cost_single,Z_SS,[],[],[],[],LB_SS,UB_SS,@constr_single,options);
Z_SS = sol_single;
U = Z_SS;

X = zeros(Nx,N+1);
X(:,1) = x0;
for k = 1:N
    u_k = U((k-1)*Nu+1:k*Nu);
    input.x = X(:,k);
    input.u = u_k;
    output = RK4_integrator( @ode, input );
    x_k = output.value;
    X(:,k+1) = x_k;
end

%% Part 2: Multiple shooting method

% Bound values:
LB_MS = -inf*ones(Nz,N);
UB_MS = inf*ones(Nz,N);
LB_MS(end,:) = -umax*ones(1,N);
UB_MS(end,:) = umax*ones(1,N);
% terminal state node
LB_MS = [LB_MS(:); -inf*ones(Nx,1)];
UB_MS = [UB_MS(:); inf*ones(Nx,1)];

% Initialize multiple shooting variables:
Z_MS = repmat([x0; 0],1,N);
Z_MS = [Z_MS(:); x0];

% Setting options and solving the problem with fmincon + MULTIPLE SHOOTING:
options = optimoptions(@fmincon,'Display','iter','Algorithm','interior-point','GradObj','on','GradConstr','on','Hessian','user-supplied','HessFcn',@hessian_multiple);
sol_multiple = fmincon(@cost_multiple,Z_MS,[],[],[],[],LB_MS,UB_MS,@constr_multiple,options);

Z_MS = sol_multiple;
X2 = zeros(Nx,N+1);
U2 = zeros(Nu,N);
for k = 1:N
   X2(:,k) = Z_MS((k-1)*Nz+1:(k-1)*Nz+Nx);
   U2(:,k) = Z_MS((k-1)*Nz+Nx+1:k*Nz);
end
X2(:,N+1) = Z_MS(N*Nz+1:end);

%% Part 3: Compare OCP solutions

figure;
subplot(321);
plot([0:N]*Ts,X(1,:),'--rx'); hold on;
plot([0:N]*Ts,X2(1,:),'--bo');
xlabel('time(s)'); ylabel('p');
legend('Single shooting', 'Multiple shooting')

subplot(322);
plot([0:N]*Ts,X(2,:),'--rx'); hold on;
plot([0:N]*Ts,X2(2,:),'--bo');
xlabel('time(s)'); ylabel('\theta');
legend('Single shooting', 'Multiple shooting')

subplot(323);
plot([0:N]*Ts,X(3,:),'--rx'); hold on;
plot([0:N]*Ts,X2(3,:),'--bo');
xlabel('time(s)'); ylabel('v');
legend('Single shooting', 'Multiple shooting')

subplot(324);
plot([0:N]*Ts,X(4,:),'--rx'); hold on;
plot([0:N]*Ts,X2(4,:),'--bo');
xlabel('time(s)'); ylabel('\omega');
legend('Single shooting', 'Multiple shooting')

subplot(3,2,[5 6]);
stairs([0:N-1]*Ts,U,'--rx'); hold on;
stairs([0:N-1]*Ts,U2,'--bo');
xlabel('time(s)'); ylabel('F');
legend('Single shooting', 'Multiple shooting')














