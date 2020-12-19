% Exercise 8 - Task 1 Modelling and System Identification
% Author: Jesus Lago Garica

function [a, b] = arx_pem(y, u, na, nb)

% Checking correctes of the data
if (length(y)~= length(u))
    error('The control data and measurement data have different size');
    return;
end
if ((na<0) || (nb<1))
    error('The size assigned to na and nb is not correct, please assign na > -1 and nb > 0');
    return;
end

% Defining the maximum size between na and nb
n = max(na,nb);
% Defining the size of y and u
N = length(y);

% Pre-allocating memory
Phi=zeros(N-n,na+nb);

% Defining Phi using the PEM method
for i=1:n
    if(i<=na)
        Phi(:,i) = -y(n+1-i:N-i);
    end
    if(i<=nb)
        Phi(:,na+i) = u(n+1-i:N-i);
    end
end

% Calculating the optimum theta = [a,b]
theta = pinv(Phi)*y(n+1:N);

% Extracting parameters
a = theta(1:na);
b=theta(na+1:na+nb);

end

