
t=t_sim;
for k=1:size(t,2)
    disp(k);
    T_m{k} = (0.2 + 0.1*sin(2*t(k)) + 0.1*sin(3*t(k)));
    T_s{k} = (0.25 + 0.1*sin(2*t(k)) + 0.05*sin(4*t(k)));
    
    f_m = 1 - (0.2*cos(2*t(k)) + 0.3*cos(3*t(k)));
    f_s = 1 - (0.2*cos(2*t(k)) + 0.2*cos(4*t(k)));
    
    delay_step_m = round(T_m{k}*fs);
    delay_step_s = round(T_s{k}*fs);

    if delay_step_m >= k
        q_s_ref{k} = [0; 0];
        dq_s_ref{k} = [0; 0];
        ddq_s_ref{k} = [0; 0];
    elseif delay_step_m< k
        q_s_ref{k} = q_m{k - delay_step_m};
        dq_s_ref{k} = dq_m{k - delay_step_m};
        ddq_s_ref{k} = ddq_m{k - delay_step_m};
    end
    
    if delay_step_s >= k
        q_m_ref{k} = [0; 0];
        dq_m_ref{k} = [0; 0];
        ddq_m_ref{k} = [0; 0];
    elseif delay_step_s < k
        q_m_ref{k} = q_s{k - delay_step_s};
        dq_m_ref{k} = dq_s{k - delay_step_s};
        ddq_m_ref{k} = ddq_s{k - delay_step_s};
    end
    
    e_m{k}=q_m_ref{k}-q_m{k};
    de_m{k}=f_m*dq_m_ref{k}-dq_m{k};
    s_m{k}=de_m{k}+lamda_m*e_m{k};
    
    e_s{k}=q_s_ref{k}-q_s{k};
    de_s{k}=f_s*dq_s_ref{k}-dq_s{k};
    s_s{k}=de_s{k}+lamda_s*e_s{k};
    % collect data
    x_m{k}=[s_m{k};e_m{k};q_m_ref{k};dq_m_ref{k}];
    x_s{k}=[s_s{k};e_s{k};q_s_ref{k};dq_s_ref{k}];
    % Use optimal control input signal 
    u_m{k}=Wu_m'*theta(x_m{k});
    u_s{k}=Wu_s'*theta(x_s{k});
    % RISE term
    integral_m=integral_m + ((ks_m+1)*alpha_m*s_m{k}+beta_m*sign(s_m{k}))*step;
    integral_s=integral_s + ((ks_s+1)*alpha_s*s_s{k}+beta_s*sign(s_s{k}))*step;
    muy_m{k}=(ks_m+1)*(s_m{k}-s_m{1}) + integral_m;
    muy_s{k}=(ks_s+1)*(s_s{k}-s_s{1}) + integral_s;
    % Input torques applied on the master and slave robot
    tor_m{k}=muy_m{k} - u_m{k};
    tor_s{k}=muy_s{k} - u_s{k};
    % Human and environmental torques 
    tor_hu{k}=torque_Hu(t,k,tor_hu);
    tor_en{k}=torque_En(t,k,tor_en);
    
    w=0.2+0.3*sin(15*t(k));
    noise=(2*rand(2,1)-[1;1]);
    ddq_m{k}=M(rho_m,q_m{k})\(-C(rho_m,q_m{k},dq_m{k})*dq_m{k}-G(rho_m,q_m{k})+tor_m{k}+tor_hu{k});
    ddq_s{k}=M(rho_s,q_s{k})\(-C(rho_s,q_s{k},dq_s{k})*dq_s{k}-G(rho_s,q_s{k})+tor_s{k}-tor_en{k});
    if k==size(t,2)
        break;
    end
    dq_m{k+1}=dq_m{k} + ddq_m{k}*step;
    q_m{k+1}=q_m{k} + dq_m{k}*step;
    
    dq_s{k+1}=dq_s{k} + ddq_s{k}*step;
    q_s{k+1}=q_s{k} + dq_s{k}*step;
end

%% PLOT
qm=cell2mat(q_m);
qmref=cell2mat(q_m_ref);
em=cell2mat(e_m);
um=cell2mat(u_m);
torm=cell2mat(tor_m);
muym=cell2mat(muy_m);
torhu=cell2mat(tor_hu);

qs=cell2mat(q_s);
qsref=cell2mat(q_s_ref);
es=cell2mat(e_s);
us=cell2mat(u_s);
tors=cell2mat(tor_s);
muys=cell2mat(muy_s);
toren=cell2mat(tor_en);
Tm=cell2mat(T_m);
Ts=cell2mat(T_s);
figure(1);
subplot(2,1,1);
plot(t,qm(1,:),t,qs(1,:),'linewidth',1.2);
legend('q_m_1','q_s_1');xlabel('Time(s)');ylabel('1^{st} joint');
subplot(2,1,2);
plot(t,qm(2,:),t,qs(2,:),'linewidth',1.2);
legend('q_m_2','q_s_2');xlabel('Time(s)');ylabel('2^{nd} joint');

figure(2);
subplot(2,1,1);
plot(t,em(1,:),t,em(2,:),'linewidth',1.2);
legend('e_m_1','e_m_2');xlabel("Time(s)");ylabel({'Tracking error';'1^{st} joint'});
subplot(2,1,2);
plot(t,es(1,:),t,es(2,:),'linewidth',1.2);
legend('e_s_1','e_s_2');xlabel("Time(s)");ylabel({'Tracking error';'2^{nd} joint'});

figure(3);
subplot(2,1,1);
plot(t,um,'linewidth',1.2);
legend('u_m_1','u_m_2');xlabel('Time(s)');ylabel({'Control input'; 'Master robot'});
subplot(2,1,2);
plot(t,us,'linewidth',1.2);
legend('u_s_1','u_s_2');xlabel('Time(s)');ylabel({'Control input'; 'Slave robot'});

figure(4);
subplot(2,1,1);
plot(t,torhu(1,:),t,torhu(2,:),'linewidth',1.2);
legend('\tau_{hu1}','\tau_{hu2}');xlabel("Time(s)");ylabel('Human torque');
subplot(2,1,2);
plot(t,toren(1,:),t,toren(2,:),'linewidth',1.2);
legend('\tau_{en1}','\tau_{en2}');xlabel("Time(s)");ylabel('Environment torque');

figure(5);
subplot(2,1,1);
plot(t,torm(1,:),t,torm(2,:),'linewidth',1.2);
legend('\tau_{m1}','\tau_{m2}');xlabel("Time(s)");ylabel('Master torque');
subplot(2,1,2);
plot(t,tors(1,:),t,tors(2,:),'linewidth',1.2);
legend('\tau_{s1}','\tau_{s2}');xlabel("Time(s)");ylabel('Slave torque');

figure(6);
subplot(2,1,1);
plot(t,torm(1,:),t,torm(2,:),'linewidth',1.2);
legend('\mu_{m1}','\mu_{m2}');xlabel("Time(s)");ylabel('Master robot');
subplot(2,1,2);
plot(t,tors(1,:),t,tors(2,:),'linewidth',1.2);
legend('\mu_{s1}','\mu_{s2}');xlabel("Time(s)");ylabel('Slave robot');

figure(7);
subplot(2,1,1);
plot(t,Tm,'linewidth',1.2);xlabel('Time(s)');ylabel('T_m');
subplot(2,1,2);
plot(t,Ts,'linewidth',1.2);xlabel('Time(s)');ylabel('T_s');

figure(8);
subplot(2,1,1);
plot(0:30,WM,'o-','linewidth',1.5);xlabel('Step of iteration');ylabel('norm(W_m)');
subplot(2,1,2);
plot(0:30,WS,'o-','linewidth',1.5);xlabel('Step of iteration');ylabel('norm(W_s)');

function f=M(rho,q)
f=[rho(1)+2*rho(2)*cos(q(2)),rho(3)+rho(2)*cos(q(2));
    rho(3)+rho(2)*cos(q(2)),rho(3)];
end

function f=G(rho,q)
f=[rho(4)*cos(q(1))+rho(5)*cos(q(1)+q(2));
    rho(5)*cos(q(1)+q(2))];
end

function f=C(rho,q,dq)
f=[-rho(2)*sin(q(2)*dq(2)),-rho(2)*sin(q(2))*(dq(1)+dq(2));
    rho(2)*sin(q(2))*dq(1),0];
end

