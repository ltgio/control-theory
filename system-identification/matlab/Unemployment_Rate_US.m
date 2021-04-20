%% Modelling unemployment rate in America
clc;clear all;close all;

y_data = [4.0	4.1	4.0	3.8	4.0	4.0	4.0	4.1	3.9	3.9	3.9	3.9... 
          4.2	4.2	4.3	4.4	4.3	4.5	4.6	4.9	5.0	5.3	5.5	5.7...
          5.7	5.7	5.7	5.9	5.8	5.8	5.8	5.7	5.7	5.7	5.9	6.0...
          5.8	5.9	5.9	6.0	6.1	6.3	6.2	6.1	6.1	6.0	5.8	5.7...
          5.7	5.6	5.8	5.6	5.6	5.6	5.5	5.4	5.4	5.5	5.4	5.4...
          5.3	5.4	5.2	5.1	5.0	5.0	4.9	5.0	5.0	5.0	5.0	4.9...
          4.7	4.8	4.7	4.7	4.6	4.6	4.7	4.7	4.5	4.4	4.5	4.4...
          4.6	4.5	4.4	4.5	4.4	4.6	4.7	4.6	4.7	4.7	4.7	5.0...
          5.0	4.9	5.1	5.0	5.4	5.6	5.8	6.1	6.1	6.5	6.8	7.3...
          7.8	8.3	8.7	9.0	9.4	9.5	9.5	9.5	9.6	9.8	10 	9.9...
          9.8	9.8	9.9	9.9	9.6	9.4	9.4	9.5	9.5	9.4	9.8	9.3...
          9.1	9.0	9.0	9.1	9.0	9.1	9.0	9.0	9.0	8.8	8.6	8.5...
          8.3	8.3	8.2	8.2	8.2	8.2	8.2	8.1	7.8	7.8	7.7	7.9...
          8.0	7.7	7.5	7.6	7.5	7.5	7.3	7.3	7.3	7.2	6.9	6.7...
          6.6	6.7 6.7	6.2	6.2	6.2	6.1	6.2	6.0	5.7	5.8	5.6...
          5.7	5.5	5.5	5.4	5.5	5.3	5.3	5.1	5.1	5.0	5.0	5.0]';

N      = length(y_data);        
t      = linspace(1,N,N)';
deltaT = t(2)-t(1);
        
figure;hold on;
title('Percentage of Unemployment in US from 2000-2015');
xlabel('number of measurement [k]');
ylabel('Unemployment Pecentage [%]');
plot1 = plot(t,y_data,'Color','b','LineWidth',2,'DisplayName','measurements');grid on;
plots = plot1;
legend(plots)

%% set Prediction Error Method
% the model is a AR: y[k] = a1*y[k-1]+a2*y[k-2]+...+an*y[k-n] + e[k]
d_max = 20;                      % max order of AR
for d = 4:2:d_max
yN    = y_data(d+1:N);          
Phi   = zeros(length(yN),d);
for i = 1:d
Phi(:,i) = y_data((d+1)-i:N-i);  
end
theta = transpose(pinv(Phi)*yN); % perform fitting
% make fitting and evaluate cost
y_fitting      = zeros(N,1);
y_fitting(1:d) = y_data(1:d);
for i = d:N-1
  y_temp = y_data(i-(d-1):i)';
  y_flip = fliplr(y_temp);
  y_pred = theta*y_flip';
  %plot(i+1,y_pred,'MarkerSize',4,'Marker','*','Color','g');
  %pause(0.05)
  y_fitting(i+1) = y_pred; 
end

cost = 0.5*(y_data-y_fitting)'*(y_data-y_fitting);
plot_fit = plot(t,y_fitting,'MarkerSize',3,'Marker','*',...
                'DisplayName',['fitting n_{a} = ',num2str(d)]);
              
              %                        '  cost = ',num2str(cost)]);
plots = [plots,plot_fit];
legend(plots)
pause(1.5)
end

%% perform prediction 4 step ahead
y_pred = y_data(end-d+1:end);

y_pred1 = theta(1)*y_pred(end)+ theta(2)*y_pred(end-1)+ theta(3)*y_pred(end-2) + theta(4)*y_pred(end-3)
y_pred2 = theta(1)*y_pred1    + theta(2)*y_pred(end)  + theta(3)*y_pred(end-1) + theta(4)*y_pred(end-2);
y_pred3 = theta(1)*y_pred2    + theta(2)*y_pred1      + theta(3)*y_pred(end)   + theta(4)*y_pred(end-1);
y_pred4 = theta(1)*y_pred3    + theta(2)*y_pred2      + theta(3)*y_pred1       + theta(4)*y_pred(end)  ;

plot(N+2,y_pred1,'mx')
plot(N+3,y_pred2,'mx')
plot(N+4,y_pred3,'mx')
plot(N+5,y_pred4,'mx')


% N_pred = 5;
% y_pred = zeros(N_pred,1);
% 
% y_pred(1:N_pred) = y_data(1:d);
% for i = d:N-1
%   y_temp = y_data(i-(d-1):i)';
%   y_flip = fliplr(y_temp);
%   y_pred = theta*y_flip';
% 
% end
% y_pred = theta*fliplr(y_pred')'

