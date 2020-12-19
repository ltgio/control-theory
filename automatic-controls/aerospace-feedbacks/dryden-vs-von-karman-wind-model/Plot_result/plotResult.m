figure
subplot(1,2,1)
plot(V_wind_d.time,V_wind_d.signals.values,'linewidth',1);title('$v_{wind}$ - Dryden Model','interpreter','latex');grid;xlabel('$t$','interpreter','latex');ylabel('$v(t)$','interpreter','latex');legend('u_{g_w}','v_{g_w}','w_{g_w}');
subplot(1,2,2)
plot(V_wind_vk.time,V_wind_vk.signals.values,'linewidth',1);title('$v_{wind}$ - Von Karman Model','interpreter','latex');grid;xlabel('$t$','interpreter','latex');ylabel('$v(t)$','interpreter','latex');legend('u_{g_w}','v_{g_w}','w_{g_w}');
figure
subplot(1,2,1)
plot(w_wind_d.time,w_wind_d.signals.values,'linewidth',1);title('$\omega_{wind}$ - Dryden Model','interpreter','latex');grid;xlabel('$t$','interpreter','latex');ylabel('$w(t)$','interpreter','latex');legend('p_{g_w}','q_{g_w}','r_{g_w}');
subplot(1,2,2)
plot(w_wind_vk.time,w_wind_vk.signals.values,'linewidth',1);title('$\omega_{wind}$ - Von Karman Model','interpreter','latex');grid;xlabel('$t$','interpreter','latex');ylabel('$w(t)$','interpreter','latex');legend('p_{g_w}','q_{g_w}','r_{g_w}');
