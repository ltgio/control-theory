function LTImodalAnalysis(A)
  [V,D] = eig(A);

  lambda = diag(D);                              % eigenvalues       [-]
  wn     = abs(lambda);                          % natural frequency [rad/s]
  tau    = 1./wn;                                % constant time     [s]
  delta  = -real(lambda)./wn;                    % damping ratio     [-]

  tp = (pi.*tau)./sqrt(1-delta.^2);              % time to first peak    [s]
  OS = exp((-pi.*delta)./sqrt(1-delta.^2)).*100; % Overshoot             [%]
  P  = (2.*pi.*tau)./sqrt(1-delta.^2);           % Period of oscillation [s]

  disp('     Eigenvalues')
  disp(lambda)
  
  disp('     Eigenvector')
  disp(V)
  
  disp('    Natural  Constant   Damping    First              Period of ')
  disp('   frequency   time      ratio     peak   Overshoot  oscillation')
  disp('    [rad/s]     [s]       [-]       [s]      [%]       [s]   ')

  disp([wn,tau,delta,tp,OS,P])
end