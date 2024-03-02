%% plot the comparision between 3 methods: OFF-policy, Finite-time controller, and Fixed-time controller
close all

figure(1)

subplot(3,1,1)
plot(t,qm(1,1:N),'r',t,qs(1,1:N),'b','linewidth',1.3)
grid on;
ylabel('1^{st} Joint (rad)');
xlabel('Time(s)');
title('OFF-Policy method')
legend('\bf{\eta_{l1}}','\bf{\eta_{r1}}','Orientation','horizontal');

subplot(3,1,2)
plot(t,qm_1(1,1:N),'r',t,qs_1(1,1:N),'b','linewidth',1.3)
grid on;
ylabel('1^{st} Joint (rad)');
xlabel('Time(s)');
title('Finite-time controller')
legend('\bf{\eta_{l1}}','\bf{\eta_{r1}}','Orientation','horizontal');

subplot(3,1,3)
plot(t,qm_2(1,1:N),'r',t,qs_2(1,1:N),'b','linewidth',1.3)
grid on;
ylabel('1^{st} Joint (rad)');
xlabel('Time(s)');
title('Fixed-time controller')
legend('\bf{\eta_{l1}}','\bf{\eta_{r1}}','Orientation','horizontal');

figure(2)

subplot(3,1,1)
plot(t,qm(2,1:N),'r',t,qs(2,1:N),'b','linewidth',1.3)
grid on;
ylabel('2^{nd} Joint (rad)');
xlabel('Time(s)');
title('OFF-Policy method')
legend('\bf{\eta_{l2}}','\bf{\eta_{r2}}','Orientation','horizontal');

subplot(3,1,2)
plot(t,qm_1(2,1:N),'r',t,qs_1(2,1:N),'b','linewidth',1.3)
grid on;
ylabel('2^{nd} Joint (rad)');
xlabel('Time(s)');
title('Finite-time controller')
legend('\bf{\eta_{l2}}','\bf{\eta_{r2}}','Orientation','horizontal');

subplot(3,1,3)
plot(t,qm_2(2,1:N),'r',t,qs_2(2,1:N),'b','linewidth',1.3)
grid on;
ylabel('2^{nd} Joint (rad)');
xlabel('Time(s)');
title('Fixed-time controller')
legend('\bf{\eta_{l2}}','\bf{\eta_{r2}}','Orientation','horizontal');


figure(3);

subplot(2,1,1);
plot(t,em(1,1:N),'r',t,em_1(1,1:N),'b',t,em_2(1,1:N),'k','linewidth',1.3);
grid on;
ylabel({'Tracking error in Local side';'1^{st} Joint (rad)'});
xlabel('Time(s)');
legend('Using OFF-Policy method','Using finite-time controller','Using fixed-time controller','Orientation','horizontal'); 

subplot(2,1,2);
plot(t,em(2,1:N),'r',t,em_1(2,1:N),'b',t,em_2(2,1:N),'k','linewidth',1.3);
grid on;
ylabel({'Tracking error in Local side';'2^{st} Joint (rad)'});
xlabel('Time(s)');
legend('Using OFF-Policy method','Using finite-time controller','Using fixed-time controller','Orientation','horizontal'); 


figure(4);

subplot(2,1,1);
plot(t,es(1,1:N),'r',t,es_1(1,1:N),'b',t,es_2(1,1:N),'k','linewidth',1.3);
grid on;
ylabel({'Tracking error in Remote side';'1^{st} Joint (rad)'});
xlabel('Time(s)');
legend('Using OFF-Policy method','Using finite-time controller','Using fixed-time controller','Orientation','horizontal');  

subplot(2,1,2);
plot(t,es(2,1:N),'r',t,es_1(2,1:N),'b',t,es_2(2,1:N),'k','linewidth',1.3);
grid on;
ylabel({'Tracking error in Remote side';'2^{st} Joint (rad)'});
xlabel('Time(s)');
legend('Using OFF-Policy method','Using finite-time controller','Using fixed-time controller','Orientation','horizontal');  

figure(5);

subplot(2,1,1);
plot(t,10*torhu(1,:),'r',t,10*toren(1,:),'b','linewidth',1.2);
legend('\tau_{hu1}','\tau_{en2}');xlabel("Time(s)");ylabel('External torque (mNm)');grid on;
subplot(2,1,2);
plot(t,10*torhu(2,:),'r',t,10*toren(2,:),'b','linewidth',1.2);
legend('\tau_{hu2}','\tau_{en2}');xlabel("Time(s)");ylabel('External torque (mNm)');grid on;


figure(6);

subplot(2,1,1);
plot(t,10*torhu_1(1,:),'r',t,10*toren_1(1,:),'b','linewidth',1.2);
legend('\tau_{hu1}','\tau_{en2}');xlabel("Time(s)");ylabel('External torque (mNm)');grid on;
subplot(2,1,2);
plot(t,10*torhu_1(2,:),'r',t,10*toren_1(2,:),'b','linewidth',1.2);
legend('\tau_{hu2}','\tau_{en2}');xlabel("Time(s)");ylabel('External torque (mNm)');grid on;



%off policy torque
figure(7);

subplot(2,1,1);
plot(t,torm(1,:),'r',t,torm(2,:),'b','linewidth',1.2);
legend('\tau_{l1}','\tau_{l2}');xlabel("Time(s)");ylabel('Local torque (mNm)');grid on;
subplot(2,1,2);
plot(t,tors(1,:),'r',t,tors(2,:),'b','linewidth',1.2);
legend('\tau_{r1}','\tau_{r2}');xlabel("Time(s)");ylabel('Remote torque (mNm)');grid on;


%IET torque
figure(8);

subplot(2,1,1);
plot(t,torm_1(1,:),'r',t,torm_1(2,:),'b','linewidth',1.2);
legend('\tau_{l1}','\tau_{l2}');xlabel("Time(s)");ylabel('Local torque (mNm)');grid on;
subplot(2,1,2);
plot(t,tors_1(1,:),'r',t,tors_1(2,:),'b','linewidth',1.2);
legend('\tau_{r1}','\tau_{r2}');xlabel("Time(s)");ylabel('Remote torque (mNm)');grid on;


%457 torque
figure(9);

subplot(2,1,1);
plot(t,torm_2(1,:),'r',t,torm_2(2,:),'b','linewidth',1.2);
legend('\tau_{l1}','\tau_{l2}');xlabel("Time(s)");ylabel('Local torque (mNm)');grid on;
subplot(2,1,2);
plot(t,tors_2(1,:),'r',t,tors_2(2,:),'b','linewidth',1.2);
legend('\tau_{r1}','\tau_{r2}');xlabel("Time(s)");ylabel('Remote torque (mNm)');grid on;



figure(10);
subplot(2,1,1);
plot(t,muym(1,:),'r',t,muym(2,:),'b','linewidth',1.2);
legend('\mu_{l1}','\mu_{l2}');xlabel("Time(s)");ylabel('Local robot');grid on;
subplot(2,1,2);
plot(t,muys(1,:),'r',t,muys(2,:),'b','linewidth',1.2);
legend('\mu_{r1}','\mu_{r2}');xlabel("Time(s)");ylabel('Remote robot');grid on;


figure(11);
subplot(2,1,1);
plot(t,um(1,1:N),'r',t,um(2,1:N),'b','linewidth',1.2);
legend('u_l_1','u_l_2');xlabel('Time(s)');ylabel({'Control input'; 'Local robot'});grid on;
subplot(2,1,2);
plot(t,us(1,1:N),'r',t,us(2,1:N),'b','linewidth',1.2);
legend('u_r_1','u_r_2');xlabel('Time(s)');ylabel({'Control input'; 'Remote robot'});grid on;


figure(12);
subplot(2,1,1);
plot(t,Tm,'linewidth',1.2);xlabel('Time(s)');ylabel('T_l (s)');grid on;
subplot(2,1,2);
plot(t,Ts,'linewidth',1.2);xlabel('Time(s)');ylabel('T_r (s)');grid on;


figure(13);
grid on
subplot(2,1,1);
plot(1:itenum,Wm,'o-','linewidth',1.5);xlabel('Step of iteration');ylabel('norm(W_l)');
ax = gca;
ax.FontSize = 10;
subplot(2,1,2);
plot(1:itenum,Ws,'o-','linewidth',1.5);xlabel('Step of iteration');ylabel('norm(W_r)');
ax = gca;
ax.FontSize = 10;


figure(14);
grid on
subplot(2,1,1);
plot(1:N,Wm_457(1:35001),'o-','linewidth',1.5);xlabel('Step of iteration');ylabel('norm(W_l)');
ax = gca;
ax.FontSize = 10;
subplot(2,1,2);
plot(1:N,Ws_457(1:35001),'o-','linewidth',1.5);xlabel('Step of iteration');ylabel('norm(W_r)');
ax = gca;
ax.FontSize = 10;



figure(15);
plot(1:35000,-costFunctionm_off(1:35000)+costFunctionm_IET,'r',1:35000,-costFunctions_off(1:35000)+costFunctions_IET,'b','linewidth',1.3);xlabel('Cost Function');
grid on;
xlabel('Time(s)');
title('J_{Finite Time} - J_{OFF Policy}')
legend('\DeltaJ_l','\DeltaJ_r','Orientation','horizontal'); 



figure(16);
plot(1:35000,-costFunctionm_off(1:35000)+costFunctionm_457,'r',1:35000,-costFunctions_off(1:35000)+costFunctions_457,'b','linewidth',1.3);xlabel('Cost Function');
grid on;
xlabel('Time(s)');
title('J_{Fixed - Time} - J_{OFF Policy}')
legend('\DeltaJ_l','\DeltaJ_r','Orientation','horizontal'); 

% figure(17)
% plot(t,qmref(1,1:N),t,qmref_1(1,1:N),t,qmref_2(1,1:N),t,qmref_3(1,1:N))
% 
% figure(18)
% plot(t,qmref(2,1:N),t,qmref_1(2,1:N),t,qmref_2(2,1:N),t,qmref_3(2,1:N))