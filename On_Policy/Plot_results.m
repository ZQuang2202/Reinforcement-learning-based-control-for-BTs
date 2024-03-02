close all;
k = 1;
ql = cell2mat(q_l);
ql_ref = cell2mat(q_l_ref);
qr = cell2mat(q_r);
qr_ref = cell2mat(q_r_ref);
t_hu = cell2mat(torque_hu);
t_en = cell2mat(torque_en);
Wll = cell2mat(Wl);
Wrr = cell2mat(Wr);
uadp_l = cell2mat(u_l);
uadp_r = cell2mat(u_r);
uRise_l = cell2mat(muy_l);
uRise_r = cell2mat(muy_r);
torquel = cell2mat(torque_l);
torquer = cell2mat(torque_r);
costFunctionl = cell2mat(costFunction_l);
costFunctionr = cell2mat(costFunction_r);

el = cell2mat(e_l);
er = cell2mat(e_r);

Tl = cell2mat(T_l);
Tr = cell2mat(T_r);

% qm_3 = cell2mat(q_l);
% qmref_3 = cell2mat(q_l_ref);
% qs_3 = cell2mat(q_r);
% qsref_3 = cell2mat(q_r_ref);
% em_3 = cell2mat(e_l);
% es_3 = cell2mat(e_r);

figure(k)
%subplot(2,1,1);
plot(t, ql(1, :),'r','linewidth', 1.4);
grid on;
hold on;
% plot(t, ql_ref(1, :),'linewidth', 1.);
plot(t, qr(1, :),'b','linewidth', 1.4);
axis([0, 35, -0.5, 1]);
legend("Local", "Remote");
ylabel("1^{st} joints (rad)");
xlabel("Time (s)");

k = k + 1;
figure(k)
%subplot(2,1,2);
plot(t, ql(2, :),'r','linewidth', 1.4);
grid on;
hold on;
% plot(t, ql_ref(2, :),'linewidth', 1.);
plot(t, qr(2, :),'b','linewidth', 1.4);
axis([0, 35, 0, 2.5]);
legend("Local", "Remote");
ylabel("2^{nd} joints (rad)");
xlabel("Time (s)");

k = k + 1;
figure(k)
subplot(2,1,1)
plot(t, el(1,:),'r','linewidth', 1.4);
grid on;
hold on;
plot(t, el(2,:),'b','linewidth', 1.4);
axis([0, 35, -0.8, 0.5]);
legend("e_l_1", "e_l_2");
ylabel(sprintf('Tracking errors of 2 joints\nLocal side(rad)'));
xlabel('Time (s)');
subplot(2,1,2)
plot(t, er(1,:),'r','linewidth', 1.4);
grid on;
hold on;
plot(t, er(2,:),'b','linewidth', 1.4);
axis([0, 35, -0.8, 0.8]);
legend("e_r_1", "e_r_2");
ylabel(sprintf('Tracking errors of 2 joints\nRemote side(rad)'));
xlabel('Time (s)');

k = k + 1;
figure(k)
subplot(2,1,1);
plot(t, Tl,'linewidth', 1.4);
grid on;
axis([0, 35, 0, 0.4]);
ylabel("T_l (s)");
xlabel("t (s)");
subplot(2,1,2);
plot(t, Tr,'linewidth', 1.4);
grid on;
axis([0, 35, 0.1, 0.4]);
ylabel("T_r (s)");
xlabel("Time (s)");

k = k+1;
figure(k)
subplot(2,1,1);
plot(t, 10*t_hu(1, :),'r','linewidth', 1.4);
grid on;
hold on;
plot(t, 10*t_en(1, :),'b','linewidth', 1.4);
grid on;
axis([0, 35, -100, 100]);
legend("\tau_{hu1}", "\tau_{en1}");
ylabel("1^{st} joints torque (Nm)");
xlabel("Time (s)");
subplot(2,1,2);
plot(t, 10*t_hu(2, :),'r','linewidth', 1.4);
grid on;
hold on;
plot(t, 10*t_en(2, :),'b','linewidth', 1.4);
grid on;
axis([0, 35, -100, 100]);
legend("\tau_{hu2}", "\tau_{en2}");
ylabel("2^{nd} joints torque (mNm)");
xlabel("Time (s)");

k = k+1;
figure(k)
subplot(2,1,1);
plot(t, uadp_l,'linewidth', 1.4);
grid on;
legend('u_{adp1}', 'u_{adp2}');
ylabel("u_{adp} (Local Robot)");
xlabel("Time (s)");
subplot(2,1,2);
plot(t, uRise_l,'linewidth', 1.4);
grid on;
legend('u_{RISE1}', 'u_{RISE2}');
ylabel("u_{RISE} (Local Robot)");
xlabel("Time (s)");

k = k+1;
figure(k)
subplot(2,1,1);
plot(t, uadp_r,'linewidth', 1.4);
grid on;
legend('u_{adp1}', 'u_{adp2}')
ylabel("u_{adp} (Remote Robot)");
xlabel("Time (s)");
axis([0, 35, -0.2, 0.12]);
subplot(2,1,2);
plot(t, uRise_r,'linewidth', 1.4);
grid on;
legend('u_{RISE1}', 'u_{RISE2}');
ylabel("u_{RISE} (Remote Robot)");
xlabel("Time (s)");

k = k+1;
figure(k)
subplot(2,1,1);
plot(t, torquel(1,:),'linewidth', 1.3, 'Color','r');
hold on;
plot(t, torquel(2,:),'linewidth', 1.3, 'Color','b');
grid on;
legend('\tau_{l1}', '\tau_{l2}')
ylabel("Local torque (mNm)");
xlabel("Time (s)");
subplot(2,1,2);
plot(t, torquer(1,:),'linewidth', 1.3, 'Color','r');
hold on;
plot(t, torquer(2,:),'linewidth', 1.3, 'Color','b');
grid on;
grid on;
legend('\tau_{r1}','\tau_{r2}');
ylabel("Remote torque (mNm)");
xlabel("Time (s)");

k = k+1;
figure(k)
plot(t, costFunctionl,'linewidth', 1.3, 'Color','r');
hold on;
plot(t, costFunctionr,'linewidth', 1.3, 'Color','b');
grid on;
legend('J_{l1}', 'J_{l2}');
ylabel("Cost Function");
xlabel("Time (s)");

W_l_plot = zeros(1,350);
W_r_plot = zeros(1,350);
for j = 1:size(t,2)
    if mod(j, 100) == 0 
        m = j/100;
        W_l_plot(m) = W_l(j);
        W_r_plot(m) = W_r(j);
    end
end
k = k+1;
figure(k);
grid on
subplot(2,1,1);
plot(1:350,W_l_plot,'o-','linewidth',1.5);xlabel('Step of iteration (x100)');ylabel('norm(W_l)');
ax = gca;
ax.FontSize = 10;
subplot(2,1,2);
plot(1:350,W_r_plot,'o-','linewidth',1.5);xlabel('Step of iteration (x100)');ylabel('norm(W_r)');
ax = gca;
ax.FontSize = 10;