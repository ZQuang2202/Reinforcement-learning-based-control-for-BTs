clear;
clc;
load('freqSetComplete.mat');
%% System dynamics
% system dimension
n=2;m=2;N=8;
% physical model
rho_m = [0.99; 0.12; 0.28; 1.02; 0.2];
rho_s = [2.14; 0.38; 0.47; 2.0; 0.48];
%rho_m=[0.56;0.23;0.19;0.97;0.18];
%rho_s=[1.63;0.42;0.44;1.95;0.5];
%% sample time
step=0.001;fs=1/step;
T_end=35;t=0:step:T_end;
T_sim=35;t_sim=0:step:T_sim;
%% controller parameters
% Optimal schemes (off-policy)
% cost function parameters
Qe=100*eye(n);R=eye(m);
Q=[Qe zeros(n,3*n);zeros(3*n,4*n)];
lamda=0.1;
lu=length(theta(ones(N,1)));
lc=length(phi(ones(N,1)));
Number_Sample=size(t,2);
lamda_m=[1 0;0 0.6];lamda_s=lamda_m;
% RISE feedback
beta_m=diag([0.1,0.1]);ks_m=diag([35,40]);alpha_m=diag([1.1,1.1]);
beta_s=beta_m;ks_s=diag([40,35]);alpha_s=alpha_m;
%% Variable declarations
q_m=cell(1,size(t,2));dq_m=q_m;ddq_m=q_m;q_s=q_m;dq_s=q_m;ddq_s=q_m;
q_m_ref=q_m;dq_m_ref=q_m;ddq_m_ref=q_m;q_s_ref=q_m;dq_s_ref=q_m;ddq_s_ref=q_m;
muy_m=q_m;muy_s=q_m;integral_m=0;integral_s=0;
e_m=q_m;s_m=q_m;de_m=q_m;e_s=q_m;de_s=q_m;s_s=q_m;
tor_m=q_m;tor_s=q_m;tor_hu=q_m;tor_en=q_m;
T_m=q_m;T_s=q_m;
u_m=q_m;u_s=q_m;
x_m=q_m;x_s=q_m;
w=q_m;
%% Initial condition
q_m{1}=[0.5;0.6];dq_m{1}=[0;0];
q_s{1}=[0.2;0.3];dq_s{1}=[0;0];
%q_m{1}=[1.1;0.9];dq_m{1}=[0;0];
%q_s{1}=[1;1.2];dq_s{1}=[0;0];
delay_step_s=q_m;
%% Collect data
disp('Start collecting data');
for k=1:size(t,2)
    disp(k);
    T_m{k} = (0.2 + 0.1*sin(2*t(k)) + 0.1*sin(3*t(k)));
    T_s{k} = (0.25 + 0.1*sin(2*t(k)) + 0.05*sin(4*t(k)));
    
    f_m = 1 - (0.2*cos(2*t(k)) + 0.3*cos(3*t(k)));
    f_s = 1 - (0.2*cos(2*t(k)) + 0.2*cos(4*t(k)));
    
    delay_step_m = round(T_m{k}*fs);
    delay_step_s{k} = round(T_s{k}*fs);

    if delay_step_m>= k
        q_s_ref{k} = [0; 0];
        dq_s_ref{k} = [0; 0];
        ddq_s_ref{k} = [0; 0];
    elseif delay_step_m< k
        q_s_ref{k} = q_m{k - delay_step_m};
        dq_s_ref{k} = dq_m{k - delay_step_m};
        ddq_s_ref{k} = ddq_m{k - delay_step_m};
    end
    
    if delay_step_s{k} >= k
        q_m_ref{k} = [0; 0];
        dq_m_ref{k} = [0; 0];
        ddq_m_ref{k} = [0; 0];
    elseif delay_step_s{k} < k
        q_m_ref{k} = q_s{k - delay_step_s{k}};
        dq_m_ref{k} = dq_s{k - delay_step_s{k}};
        ddq_m_ref{k} = ddq_s{k - delay_step_s{k}};
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
    % given a admissible control signal
    u_m{k}=- C(rho_m,q_m{k},dq_m{k})*10*s_m{k};
    u_s{k}=- C(rho_m,q_m{k},dq_m{k})*10*s_s{k};
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
    % noise
    noise=0;
    if k<3000
    for j=1:20
        noise=noise+1/2*sin(freqSet(1,j)*t(k));
    end
    %noise=(4*rand(2,1)-2*[1;1]);
    end
    % disterabance
    %w=0.2+0.3*sin(15*t(k));
    ddq_m{k}=M(rho_m,q_m{k})\(-C(rho_m,q_m{k},dq_m{k})*dq_m{k}-G(rho_m,q_m{k})+tor_m{k}+tor_hu{k}+noise);
    ddq_s{k}=M(rho_s,q_s{k})\(-C(rho_s,q_s{k},dq_s{k})*dq_s{k}-G(rho_s,q_s{k})+tor_s{k}-tor_en{k}+noise);
     if k==size(t,2)
        break;
    end
    dq_m{k+1}=dq_m{k} + ddq_m{k}*step;
    q_m{k+1}=q_m{k} + dq_m{k}*step;
    
    dq_s{k+1}=dq_s{k} + ddq_s{k}*step;
    q_s{k+1}=q_s{k} + dq_s{k}*step;
end
%% Find optimal weights of control law and cost function
dentaV_m=cell(Number_Sample,1);dentaV_s=dentaV_m;
ixx_m=cell(Number_Sample,1);ixx_s=ixx_m;
itheta_m=cell(Number_Sample,1);itheta_s=itheta_m;
itu_m=cell(Number_Sample,1);itu_s=itu_m;
iphi_m=cell(Number_Sample,1);iphi_s=iphi_m;

% Setting up a system of equation
for k=1:Number_Sample
    disp(k);
    dentaV_m{k}=-phi(x_m{k+1})'+phi(x_m{k})';
    dentaV_s{k}=-phi(x_s{k+1})'+phi(x_s{k})';
    ixx_m{k}=0.5*step*(x_m{k+1}'*Q*x_m{k+1}+x_m{k}'*Q*x_m{k});
    ixx_s{k}=0.5*step*(x_s{k+1}'*Q*x_s{k+1}+x_s{k}'*Q*x_s{k});
    itheta_m{k}=0.5*step*(kron(theta(x_m{k})',theta(x_m{k})') + kron(theta(x_m{k+1})',theta(x_m{k+1})'));
    itheta_s{k}=0.5*step*(kron(theta(x_s{k})',theta(x_s{k})') + kron(theta(x_s{k+1})',theta(x_s{k+1})'));
    iphi_m{k}=lamda*0.5*step*(phi(x_m{k})'+phi(x_m{k+1})');
    iphi_s{k}=lamda*0.5*step*(phi(x_s{k})'+phi(x_s{k+1})');
    itu_m{k}=0.5*step*(kron(u_m{k}',theta(x_m{k})')+kron(u_m{k+1}',theta(x_m{k+1})'));
    itu_s{k}=0.5*step*(kron(u_s{k}',theta(x_s{k})')+kron(u_s{k+1}',theta(x_s{k+1})'));
    if k==Number_Sample-1
        break;
    end
end
%Initial value of weights
Wc_m0=zeros(lc,1);Wc_m=Wc_m0;
Wu_m0= zeros(lu,2);
Wu_m=Wu_m0;
Wc_s0=zeros(lc,1);Wc_s=Wc_s0;
Wu_s0= zeros(lu,2);
Wu_s=Wu_s0;
W_m=[Wc_m;Wu_m(:)];
W_s=[Wc_s;Wu_s(:)];
%Converting to matrix
dentaVm=cell2mat(dentaV_m);
dentaVs=cell2mat(dentaV_s);
ixxm=cell2mat(ixx_m);
ixxs=cell2mat(ixx_s);
ithetam=cell2mat(itheta_m);
ithetas=cell2mat(itheta_s);
iphim=cell2mat(iphi_m);
iphis=cell2mat(iphi_s);
itum=cell2mat(itu_m);
itus=cell2mat(itu_s);
WM=zeros(31,1);
WS=zeros(31,1);
% Start the iteration to find optimal weight
for k=1:31
    disp(k);
    %Master robot
    A_m=Wu_m*R*Wu_m';
    Y_m=ixxm+ithetam*A_m(:);
    H_m=[dentaVm+iphim,2*ithetam*kron(Wu_m,eye(lu))*kron(R,eye(lu))-2*itum*kron(R,eye(lu))];
    W_m=H_m\Y_m;
    Wc_m=W_m(1:lc);
    WM(k)=norm(W_m);
    Wu_m=reshape(W_m(lc+1:end),lu,m);
    disp(norm(H_m*W_m-Y_m));
    
    %Slave robot
    A_s=Wu_s*R*Wu_s';
    Y_s=ixxs+ithetas*A_s(:);
    H_s=[dentaVs+iphis,2*ithetas*kron(Wu_s,eye(lu))*kron(R,eye(lu))-2*itus*kron(R,eye(lu))];
    W_s=H_s\Y_s;
    WS(k)=norm(W_s);
    Wu_s=reshape(W_s(lc+1:end),lu,m);
    Wc_s=W_s(1:lc);
    disp(norm(H_s*W_s-Y_s));
end
%% Applying optimal weights to the systems
Apply_and_plot;

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





    


















