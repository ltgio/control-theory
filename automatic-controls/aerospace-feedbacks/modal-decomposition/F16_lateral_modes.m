%% F-16 Longitudinal Modes page 206 [bug free!!!]
% more info second order system on
% https://www.et.byu.edu/~tom/classes/436/ClassNotes/Class20(Second-Order).pdf
clc;clear all;close all;

disp('F-16 Longitudinal Modes');
%        vt     ,  alpha ,   theta   ,     q    
Along = [-2.0244e-2 , 7.8763 , -3.2170e1 , -6.5020e-1;
         -2.5372e-4 ,-1.0190 ,  0.0      ,  9.0484e-1;
          0.0       , 0.0    ,  0.0      ,  1.0      ;
          7.9472e-11,-2.4982 ,  0.0      , -1.3861   ];

LTImodalAnalysis(Along)

%% F-16 Lateral-Directional Modes page 207 [bug free!!!]
%       beta  ,    phi    ,   p       ,     r
disp('F-16 Longitudinal Modes');

Alat = [-3.22e-1 , 6.4032e-2 , 3.8904e-2 , -9.9156e-1;  % beta
         0.0     , 0.0       , 1.0       ,  3.9385e-2;  % phi
        -3.0919e1, 0.0       ,-3.6730    ,  6.7425e-1;  % p
         9.4724  , 0.0       ,-2.6358e-2 , -4.9849e-1]; % r 
  
LTImodalAnalysis(Alat)