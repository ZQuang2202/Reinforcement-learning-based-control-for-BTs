
% Simulation for "Finite-time convergence for bilateral teleoperation systems with
% disturbance and time-varying delays"
%% 
n=2;m=2;
% physical model
theta_m = [0.99; 0.12; 0.28; 1.02; 0.2];
theta_s = [2.14; 0.38; 0.47; 2.0; 0.48];

%% sample time
step=0.001;fs=1/step;
T_end=35;t=0:step:T_end;

%% control parameters
kt=15;ks=45;kd=5;beta=0.1;lamda=5;gamma=0.2;b=0.5;
Theta_m0=[0.56, 0.23, 0.19, 0.97, 0.18]';
Theta_s0=[1.63, 0.42, 0.44, 1.95, 0.5]';

%% Variable declarations
% coornadinate joints
q_m=cell(1,size(t,2));dq_m=q_m;ddq_m=q_m;q_s=q_m;dq_s=q_m;ddq_s=q_m;
q_m_ref=q_m;dq_m_ref=q_m;ddq_m_ref=q_m;q_s_ref=q_m;dq_s_ref=q_m;ddq_s_ref=q_m;
% tracking error
e_m=q_m;s_m=q_m;de_m=q_m;
e_s=q_m;s_s=q_m;de_s=q_m;
% torque
tor_m=q_m;tor_s=q_m;tor_hu=q_m;tor_en=q_m;
% time delay
T_m=q_m;T_s=q_m;
u_m=q_m;u_s=q_m;

%% Initial condition
q_m{1}=[0.5;0.6];dq_m{1}=[0;0];
q_s{1}=[-0.2;0.3];dq_s{1}=[0;0];

%% Cost function parameters
Q = [100 0; 0 100];
R = [1 0; 0 1];
costFunction_m_IET = cell(1,size(t,2));
costFunction_s_IET = cell(1,size(t,2));
N = size(t,2);
%% 
for k=1:size(t,2)

    T_m{k} = (0.2 + 0.1*sin(2*t(k)) + 0.1*sin(3*t(k)));
    T_s{k} = (0.25 + 0.1*sin(2*t(k)) + 0.05*sin(4*t(k)));
    f_m = 1 - (0.2*cos(2*t(k)) + 0.3*cos(3*t(k)));
    f_s = 1 - (0.2*cos(2*t(k)) + 0.2*cos(4*t(k)));

    delay_step_m = round(T_m{k}*fs);
    delay_step_s = round(T_s{k}*fs);

    if delay_step_m>= k
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

    e_m{k} = q_m_ref{k} - q_m{k};
    de_m{k} = f_m*dq_m_ref{k} - dq_m{k};
    s_m{k} = -lamda*e_m{k} + dq_m{k};
    
    e_s{k} = q_s_ref{k} - q_s{k};
    de_s{k} = f_s*dq_s_ref{k} - dq_s{k};
    s_s{k} = -lamda*e_s{k} + dq_s{k};

    % Human and environmental torques 

    tor_hu{k} = torque_Hu(t, k , tor_hu);
    tor_en{k} = torque_En(t, k , tor_en);

    Y_m=Y(q_m{k},dq_m{k},e_m{k},de_m{k});
    Y_s=Y(q_s{k},dq_s{k},e_s{k},de_s{k});
    
    if norm(Y_m'*s_m{k})>beta
        dTheta_m=-gamma*Y_m'*s_m{k}/norm(Y_m'*s_m{k});
    else
        dTheta_m=-gamma/beta*Y_m'*s_m{k};
    end
    if norm(Y_s'*s_s{k})>beta
        dTheta_s=-gamma*Y_s'*s_s{k}/norm(Y_s'*s_s{k});
    else
        dTheta_s=-gamma/beta*Y_s'*s_s{k};
    end
    
    Theta_m=Theta_m0 + dTheta_m;
    Theta_s=Theta_s0 + dTheta_s;

    tor_m{k}=Y_m*Theta_m - kt*s_m{k} + kd*de_m{k} - ks*sig(s_m{k},b);
    tor_s{k}=Y_s*Theta_s - kt*s_s{k} + kd*de_s{k} - ks*sig(s_s{k},b);
    
    % Disturbance:
    w = 0.3 + 0.3*sin(t(k)) + 0.3*sin(20*t(k)) - 0.2*sin(10*t(k)) + 0.3*sin(21*t(k));
    
    ddq_m{k}=M(theta_m,q_m{k})\(tor_m{k} + 10*tor_hu{k} + w - G(theta_m,q_m{k}) - C(theta_m,q_m{k},dq_m{k})*dq_m{k});
    ddq_s{k}=M(theta_s,q_s{k})\(tor_s{k} - 10*tor_en{k} + w - G(theta_s,q_s{k}) - C(theta_s,q_s{k},dq_s{k})*dq_s{k});

    if k==size(t,2)
        break;
    end

    dq_m{k+1}=dq_m{k} + ddq_m{k}*step;
    q_m{k+1}=q_m{k} + dq_m{k}*step;
    
    dq_s{k+1}=dq_s{k} + ddq_s{k}*step;
    q_s{k+1}=q_s{k} + dq_s{k}*step;
end
gamma = 0.01;
for k = 1:N-1
    costFunction_m_IET{k} = cumsum(exp(-gamma*t(k))*(0.5*e_m{k}'*Q*e_m{k} + 0.5*tor_m{k}'*R*tor_m{k}));
    costFunction_s_IET{k} = cumsum(exp(-gamma*t(k))*(0.5*e_s{k}'*Q*e_s{k} + 0.5*tor_s{k}'*R*tor_s{k}));
end
%% Converting cell to matrix
qm_1=cell2mat(q_m);
qmref_1=cell2mat(q_m_ref);
em_1=cell2mat(e_m);
torm_1=cell2mat(tor_m);
torhu_1=cell2mat(tor_hu);
costFunctionm_IET = cell2mat(costFunction_m_IET);

qs_1=cell2mat(q_s);
qsref_1=cell2mat(q_s_ref);
es_1=cell2mat(e_s);
tors_1=cell2mat(tor_s);
toren_1=cell2mat(tor_en);
Tm=cell2mat(T_m);
Ts=cell2mat(T_s);
costFunctions_IET = cell2mat(costFunction_s_IET);
%%
function f=Y(q,dq,e,de)
lamda=5;
f=[lamda*de(1),lamda*cos(q(2))*(2*de(1)+de(2))-lamda*sin(q(2))*(e(1)*dq(2)+dq(1)*e(2)+dq(2)*e(2)),lamda*de(2),9.8*cos(q(1)),9.8*cos(q(1)+q(2));
   0,lamda*cos(q(2))*de(1)+lamda*sin(q(2))*dq(1)*e(1),lamda*(de(2)+de(1)),0,9.8*cos(q(1)+q(2))];
end
function f=M(theta,q)
f=[theta(1)+2*theta(2)*cos(q(2)),theta(3)+theta(2)*cos(q(2));
    theta(3)+theta(2)*cos(q(2)),theta(3)];
end
function f=G(theta,q)
f=[theta(4)*9.8*cos(q(1))+theta(5)*9.8*cos(q(1)+q(2));
    theta(5)*9.8*cos(q(1)+q(2))];
end
function f=C(theta,q,dq)
f=[-theta(2)*sin(q(2)*dq(2)),-theta(2)*sin(q(2))*(dq(1)+dq(2));
    theta(2)*sin(q(2))*dq(1),0];
end

