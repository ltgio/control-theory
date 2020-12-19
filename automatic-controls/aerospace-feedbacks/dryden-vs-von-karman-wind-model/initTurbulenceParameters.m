function turbulenceParameters = initTurbulenceParameters( altitude , turbulenceParameters , modeTurbolence )
%initTurbulenceParameters returns scale lengths and turbulence intensities
%parameters depending of altitude of aircraft and also of turbolence severity
%in case of low altitude (h<1000ft)

%   Form: initTurbulenceParameters( altitude, modeTurb )
%   Inputs :    altitude             -> altitude of aircraft [m]
%               modeTurbolence       -> turbolence severity
%               turbolenceParameters -> structure of parameters 
%   Outputs :   turbolenceParameters -> Lu,Lv,Lw,sigma_u,sigma_v,sigma_w

%convert altitude from meters to feets
altitude=distdim(altitude,'m','ft');

%% LOW ALTITUDE (h<1000ft)
switch modeTurbolence
    case 'Light'
     W20=15;
    case 'Moderate'
     W20=30;
    case 'Heavy'
     W20=45;
end
Lw=altitude;
Lu=altitude/((0.177+0.000823*altitude)^(1.2));
Lv=Lu;

sigma_w=0.1*W20;
sigma_u=sigma_w/((0.177+0.000823*altitude)^(0.4));
sigma_v=sigma_u;

turbulenceParameters.Lu=Lu;
turbulenceParameters.Lv=Lv;
turbulenceParameters.Lw=Lw;
turbulenceParameters.sigma_u=sigma_u;
turbulenceParameters.sigma_v=sigma_v;
turbulenceParameters.sigma_w=sigma_w;
end

