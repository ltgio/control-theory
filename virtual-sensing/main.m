
n=3;      %number of state
q=0.1;    %std of process 
r=0.1;    %std of measurement
Q=q^2*eye(n); % covariance of process
R=r^2;        % covariance of measurement  
f=@(x)[x(2);x(3);0.05*x(1)*(x(2)+x(3))];  % nonlinear state equations
h=@(x)x(1);                               % measurement equation
A=[0 1 0;0 0 1;0.05 0 0];
C=[1 0 0];


s=[0;0;1];                                % initial state
x=s+q*randn(3,1); %initial state          % initial state with noise
x2=x;
P = eye(n);   
P2=P;% initial state covraiance 

N=300;                                     % total dynamic steps
xV = zeros(n,N);          %estmate        % allocate memory
sV = zeros(n,N);          %actual
zV = zeros(1,N);

xV2 = zeros(n,N);  

L=numel(x);                                 %numer of states
% m=numel(z);                                 %numer of measurements
alpha  = 1e-3;   % first scaling parameter
beta   = 2;   % second scalinng parameter for gaussian distribution
k      = 0;   % third scaling parameter
lambda = alpha^2*(L+k)-L; 
wm     = ones(2*L+1,1)*1/(2*(L+lambda)); % weight vector 
wc     = wm; 
wm(1)  = lambda/(L+lambda);
wc(1)  = lambda/(L+lambda) +1-alpha^2+beta;

% My UKF
for k=1:N
  z = C*s + r*randn;                     % measurments
  sV(:,k)= s;                             % save actual state
  zV(k)  = z;                             % save measurment
  [ x, P ,chi] = ukfilter( x,z,f,C,P,Q,R,wm,wc,lambda );
  [x2, P2] = ukf(f,x2,P2,h,z,Q,R);            % ukf 
  xV(:,k) = x;                            % save estimate
  xV2(:,k)=x2;
  s = A*s + q*randn(3,1);                % update process 
end

figure
for k=1:3                                 % plot results
  subplot(3,1,k)
  plot(1:N, sV(k,:), 1:N, xV(k,:),1:N, xV2(k,:),'-.')
  if k==1
    title('Validation UKF Algorithm','interpreter','latex');
    legend('x(1)_{real}','x(1)_{Implemented}','x(1)_{Matlab}')
    ylabel('$x(1)$','interpreter','latex');
  elseif k==2
    legend('x(2)_{real}','x(2)_{Implemented}','x(2)_{Matlab}')
    ylabel('$x(2)$','interpreter','latex');
  else
    legend('x(3)_{real}','x(3)_{Implemented}','x(3)_{Matlab}')
    ylabel('$x(3)$','interpreter','latex');
  end
  grid;xlabel('$t [s]$','interpreter','latex');
end

for k=1:length(xV)
error = (xV(:,k)-xV2(:,k));
end
errorMean=mean(error,2)