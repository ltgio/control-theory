clear all;clc;

display('*************** TURBOLENCE GENERATION SCRIPT FOR LOW ALTITUDE (h<1000 ft or 304 meters) ****************')
display('***************************** COMPARING DRYDEN MODEL & VON KARMAN MODEL*********************************')
Va=input('Choose aircraft airspeed [m/s]\nVa=');
if(Va==0)
  error('ErrorTests:convertTest','Error on speed of aircraft. Choose speed different to 0')
end
h=200; %aircraft altitude [meters]
turbolenceParameters         = struct;
turbolenceParameters.Lu      = 0;
turbolenceParameters.Lv      = 0;
turbolenceParameters.Lw      = 0;
turbolenceParameters.sigma_u = 0;
turbolenceParameters.sigma_v =0;
turbolenceParameters.sigma_w =0;
turbolenceParameters.b       =5.5;
severity=input('Choose turbolence severity between cases:\n1 Light\n2 Moderate\n3 Heavy\nSeverity n°:');
switch severity    
    case 1
     severityChoosed = 'Light'
    case 2
     severityChoosed = 'Moderate'
    case 3
     severityChoosed = 'Heavy'
otherwise 
     error('ErrorTests:convertTest','Error on turbolence severity. Choose between cases:\n> Light\n> Moderate\n> Heavy')
end
turbolenceParameters=initTurbulenceParameters(h,turbolenceParameters,severityChoosed)

Simulink.Bus.createObject(turbolenceParameters)
myParamsType = slBus1;
clear slBus1
myParams = Simulink.Parameter(turbolenceParameters);
myParams.DataType = 'Bus: myParamsType';

model=input('Choose turbolence simulation between :\n1 Dryden\n2 Von-Karman\n3 Comparing Dryden VS Von-Karmman\nSimulation chosed:');
switch model    
    case 1
     sim('turbulence_DrydenModel');
     figure(1)
     plot(V_wind_d.time,V_wind_d.signals.values,'linewidth',1);title('V wind - Dryden Model');grid;xlabel('$t$','interpreter','latex');ylabel('$v(t)$','interpreter','latex');legend('u','v','w');
     figure(2)
     plot(w_wind_d.time,w_wind_d.signals.values,'linewidth',1);title('W wind - Dryden Model');grid;xlabel('$t$','interpreter','latex');ylabel('$w(t)$','interpreter','latex');legend('p','q','r');
    case 2
     sim('turbulence_VonKarmanModel');
     figure(1)
     plot(V_wind_vk.time,V_wind_vk.signals.values,'linewidth',1);title('V wind - Von Karman Model');grid;xlabel('$t$','interpreter','latex');ylabel('$v(t)$','interpreter','latex');legend('u','v','w');
     figure(2)
     plot(w_wind_vk.time,w_wind_vk.signals.values,'linewidth',1);title('W wind - Von Karman Model');grid;xlabel('$t$','interpreter','latex');ylabel('$w(t)$','interpreter','latex');legend('p','q','r');
    case 3
     sim('turbulence_DrydenModel');
     sim('turbulence_VonKarmanModel');
     figure(1)
     subplot(1,2,1)
     plot(V_wind_d.time,V_wind_d.signals.values,'linewidth',1);title('V wind - Dryden Model');grid;xlabel('$t$','interpreter','latex');ylabel('$v(t)$','interpreter','latex');legend('u','v','w');
     subplot(1,2,2)
     plot(V_wind_vk.time,V_wind_vk.signals.values,'linewidth',1);title('V wind - Von Karman Model');grid;xlabel('$t$','interpreter','latex');ylabel('$v(t)$','interpreter','latex');legend('u','v','w');
     figure(2)
     subplot(1,2,1)
     plot(w_wind_d.time,w_wind_d.signals.values,'linewidth',1);title('W wind - Dryden Model');grid;xlabel('$t$','interpreter','latex');ylabel('$w(t)$','interpreter','latex');legend('p','q','r');
     subplot(1,2,2)
     figure(2)
     plot(w_wind_vk.time,w_wind_vk.signals.values,'linewidth',1);title('W wind - Von Karman Model');grid;xlabel('$t$','interpreter','latex');ylabel('$w(t)$','interpreter','latex');legend('p','q','r');
otherwise 
     error('ErrorTests:convertTest','Error on turbolence simulation. Choose between model:\n> Dryden\n> Von-Karman\n> Comparing Dryden VS Von-Karmman')
end