function [ x_est, Pprec,chi ] = ukfilter( x_est_prec,y,f,C,P,Q,R,wm,wc,lambda )
 nx=3; 
% Step 1: generate sigma points
  sP = chol(P,'lower'); % compute sqrt of covariance
  chi = [x_est_prec, x_est_prec*ones(1,nx)+sqrt(nx+lambda)*sP, ...
         x_est_prec*ones(1,nx)-sqrt(nx+lambda)*sP];
  % Step 2: Prediction Transformation
  
  for k=1:size(chi,2)                   
    chi_pred(:,k)=f(chi(:,k));        
  end
%   chi_pred = A*chi;
  x_mean = chi_pred*wm;      % compute mean of predicted state
  P_mean = Q;                % compute covariance of predicted state
  for i = 1:2*nx+1
    P_mean = P_mean + wc(i)*(chi_pred(:,i) - x_mean)*(chi_pred(:,i) - x_mean)';
  end
  % Step 3: Observation Transformation
  psi = C*chi_pred;
  y_mean = psi*wm;      % compute mean of predicted output
  Pyy = R;              % compute covariance of predicted output
  Pxy = zeros(nx,1);    % compute cross-covariance between state and output
  for i = 1:2*nx+1
    Pyy = Pyy + wc(i)*(psi(:,i) - y_mean)*(psi(:,i) - y_mean)';
    Pxy = Pxy + wc(i)*(chi_pred(:,i) - x_mean)*(psi(:,i) - y_mean)';
  end
  % Step nx: Measurement Update
  K = Pxy/Pyy;                               % Calculate Kalman gain
  x_est = x_mean + K*(y - y_mean);           % Update state estimate
  Pprec = P_mean - K*Pyy*K';                 % Update covariance estimate
end