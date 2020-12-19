% Example 4.4-3: A Roll Damper/Yaw Damper Design (pg. 298)
clc;clear all;close all;

%% Model
ap = [ -0.13150, 0.14858,  0.32434  , -0.93964;
       0.0    , 0.0    ,  1.0      ,  0.33976;
     -10.614  , 0.0    , -1.1793   ,  1.00230;
       0.99655, 0.0    , -0.0018174, -0.25855];

bp = [ 0.00012049 , 0.00032897;
      0.0        , 0.0       ;
     -0.1031578  , 0.020987  ;
     -0.0021330  ,-0.010715  ];
 
cp = [0.0, 0.0, rad2deg(1),    0.0     ;
     0.0, 0.0,    0.0    , rad2deg(1)];

dp = zeros(2,2);
 
plant = ss(ap,bp,cp,dp,...
     'name'     ,'Longitudinal Dynamic F16',...
     'StateName',{'beta' , 'phi' ,   'p'   ,   'r'   },...
     'StateUnit',{'[rad]','[rad]','[rad/s]','[rad/s]'},...
     'InputName',{ 'da'  , 'dr'  },...
     'InputUnit',{'[rad]','[rad]'},...
     'OutputName',{   'p'   ,  'r'},...
     'OutputUnit',{'[deg/s]','[deg/s]'}); 

clear ap bp cp dp
%% Actuator model (1st order model)
aa = [-20.0,  0.0;
        0.0,-20.0];
ba = [ 20.0,  0.0;
        0.0, 20.0];
ca = [ -1.0, 0.0;
        0.0,-1.0]; % SIGN CHANGE
da = zeros(2,2);

actua = ss(aa,ba,ca,da,...
           'name'      ,'Actuator',...
           'StateName' ,{ 'xda' , 'xdr' },...
           'StateUnit' ,{'[rad]','[rad]'},...
           'InputName' ,{ 'ua'  , 'ur'  },...
           'InputUnit' ,{'[rad]','[rad]'},...
           'OutputName',{ 'da'  , 'dr'  },...
           'OutputUnit',{'[rad]','[rad]'}); 
clear aa ba ca da
 
%% wash out filter
tauW = 1;
aw = [-1/tauW];
bw = [0 , 1/tauW];
cw = [0 ; -1];
dw = eye(2);    % y1=p , y2= washed-r
wash = ss(aw,bw,cw,dw,...
       'name'      ,'wash-out filter',...
       'StateName' ,{  'xw'   },...
       'StateUnit' ,{'[deg/s]'},...
       'InputName' ,{   'p'   ,   'r'   },...
       'InputUnit' ,{'[deg/s]','[deg/s]'},...
       'OutputName',{ 'pw'  , 'rw'  },...
       'OutputUnit',{'[deg/s]','[deg/s]'}); 
   
%% connect blocks
sys1 = series(actua,plant);
sys2 = series(sys1,wash);

%% rootlocus p/ua
[a,b,c,d] = ssdata(sys2);
k = linspace(0,0.9,3000);
r = rlocus(a,b(:,1),c(1,:),0,k);
plot(r);
grid on
axis([-12, 1,-5,5])

%% close roll loop
kp = 0.2;
acl1 = a - b(:,1)*kp*c(1,:); 
[z,p,k1] = ss2zp(acl1,b(:,2),c(2,:),0); % yaw tr. fn. + wash
k = linspace(0,9.0,3000);
r = rlocus(acl1,b(:,2),c(2,:),0,k);
figure;
plot(r) % Yaw channel root locus
grid on

%% close  roll and yaw
kp = 0.4;
kr = 1.3;
acl2 = a - b*[kp, 0; 0 ,kr]*c;

%%
t = [0:0.02:10];
u = [-1.0*ones(1,51),1.0*ones(1,50),zeros(1,400)]; % doublet
[y_noSAS,x] = lsim(a,b(:,1),c(1,:),0,u,t); % linear simulation
[y_SAS  ,x] = lsim(acl2,b(:,1),c(1,:),0,u,t); % linear simulation

plot(t,y_noSAS,t,y_SAS,t,u,'LineWidth',2.0);
grid on;axis equal


%% try LQR
% x = [xw , beta , phi , p , r , xda , xdr] 
% u = [ua, ur]
Q = diag([0,0,0,1,1,0,0]);
R = diag([1,1]);

Klq = lqr(sys2.a,sys2.b,Q,R)

