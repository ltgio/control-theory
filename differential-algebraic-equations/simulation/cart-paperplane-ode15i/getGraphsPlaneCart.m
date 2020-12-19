function [ output_args ] = getGraphsPlaneCart(tsim,Xsim)

xPlane       = Xsim(:,1);         
zPlane       = Xsim(:,2);          
xCart        = Xsim(:,3); 
vxPlane      = Xsim(:,4);
vzPlane      = Xsim(:,5);
vxCart       = Xsim(:,6);
cableLenght  = Xsim(:,7);

zCart        = 5*ones(length(Xsim),1);
vzCart       = zeros(length(Xsim),1);

figure;


end