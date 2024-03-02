
% Simulation for "Adaptive Neural Network Fixed-Time Control
% Design for Bilateral Teleoperation With Time Delay"

%% Simulation parameters
% close all;
T = 0.001;
fs = 1/T;
T_end = 35;
t = 0:T:T_end;
%% Model parameters
ro_m = [0.99; 0.12; 0.28; 1.02; 0.2];
ro_s = [2.14; 0.38; 0.47; 2.0; 0.48];

%% Control Constant
k_m21 = 2; k_s21 = 2;
k_m11 = 1.5; k_s11 = 1.1;
k_m12 = 2; k_s12 = 1;
k_m22 = 2; k_s22 = 1;
eta_k = [0.2;-0.30];
lamda_m = 1;
lamda_s = 1;
miu1_m = 0.1;
miu1_s = 0.1;
miu2_m = 0.1;
miu2_s = 0.1;

%% Variables cell 
T_m = cell(1,size(t,2));
T_s = cell(1,size(t,2));
q_s_ref = cell(1,size(t,2)); q_s_ref_dot = cell(1,size(t,2)); q_s_ref_dot_dot = cell(1,size(t,2));
q_m = cell(1,size(t,2)); q_m_dot = cell(1,size(t,2)); q_m_dot_dot = cell(1,size(t,2));
q_m_ref = cell(1,size(t,2)); q_m_ref_dot = cell(1,size(t,2)); q_m_ref_dot_dot = cell(1,size(t,2));
q_s = cell(1,size(t,2)); q_s_dot = cell(1,size(t,2)); q_s_dot_dot = cell(1,size(t,2));
z1_m = cell(1,size(t,2)); z1_s = cell(1,size(t,2)); z2_m = cell(1,size(t,2)); z2_s = cell(1,size(t,2));
D_hat_m = cell(1,size(t,2)); D_hat_s = cell(1,size(t,2));
W_m = cell(1,size(t,2)); W_s = cell(1,size(t,2));
tau_m = cell(1,size(t,2)); tau_s = cell(1,size(t,2));
torque_hu = cell(1,size(t,2)); torque_en = cell(1,size(t,2));
costFunction_m = cell(1,size(t,2));
costFunction_s = cell(1,size(t,2));

%% Neural network
l = 10;
W_m{1} = zeros(l,2);
W_s{1} = zeros(l,2);
Gamma_m = [1 0;0 1];
Gamma_s = Gamma_m;
sigma1_m = 0.5;
sigma1_s = 1;
sigma2_m = 0.5;
sigma2_s = 1;

%% Initial 
q_m{1} = [0.5;0.6];
% q_m{1} = [1.1;0.9];
q_m_dot{1} = [0;0];
q_s{1} = [-0.2;0.3];
% q_s{1} = [1;1.2];
q_s_dot{1} = [0;0];
D_hat_m{1} = 0;
D_hat_s{1} = 0;

%% computing the cost function
Q = [100,0;0,100];
R = [1 0; 0 1];
%% Loop
for k = 1:size(t,2)
    
    T_m{k} = (0.2 + 0.1*sin(2*t(k)) + 0.1*sin(3*t(k)));
    T_s{k} = (0.25 + 0.1*sin(2*t(k)) + 0.05*sin(4*t(k)));
    
    delay_step_m = round(T_m{k}*fs);
    delay_step_s = round(T_s{k}*fs);
    
    if delay_step_m>= k
        q_s_ref{k} = [0; 0];
        q_s_ref_dot{k} = [0; 0];
        q_s_ref_dot_dot{k} = [0; 0];
    elseif delay_step_m< k
        q_s_ref{k} = q_m{k - delay_step_m};
        q_s_ref_dot{k} = q_m_dot{k - delay_step_m};
        q_s_ref_dot_dot{k} = q_m_dot_dot{k - delay_step_m};
    end
    
    if delay_step_s >= k
        q_m_ref{k} = [0; 0];
        q_m_ref_dot{k} = [0; 0];
        q_m_ref_dot_dot{k} = [0; 0];
    elseif delay_step_s < k
        q_m_ref{k} = q_s{k - delay_step_s};
        q_m_ref_dot{k} = q_s_dot{k - delay_step_s};
        q_m_ref_dot_dot{k} = q_s_dot_dot{k - delay_step_s};
    end
    
    r_m = q_m_dot{k} - q_m_ref_dot{k};
    r_s = q_s_dot{k} - q_s_ref_dot{k};
    
    z1_m{k} = q_m{k} - q_m_ref{k};
    z1_s{k} = q_s{k} - q_s_ref{k};
    
    alpha_m = -0.5^0.75*k_m11*transposePlus(z1_m{k})*(z1_m{k}'*z1_m{k})^0.75 - 0.5^2*k_m12*z1_m{k}*z1_m{k}'*z1_m{k};
    alpha_s = -0.5^0.75*k_s11*transposePlus(z1_s{k})*(z1_s{k}'*z1_s{k})^0.75 - 0.5^2*k_s12*z1_s{k}*z1_s{k}'*z1_s{k};
    
    z2_m{k} = r_m - alpha_m;
    z2_s{k} = r_s - alpha_s;
    
    fz_m = k_m11*(-0.5^0.75*(z1_m{k}'*z1_m{k})^(-5/4) + 0.5^(7/4)*3*z1_m{k}*z1_m{k}'*(z1_m{k}'*z1_m{k})^(9/4)) - 0.25*k_m12*(2*z1_m{k}*z1_m{k}' + z1_m{k}'*z1_m{k});
    fz_s = k_s11*(-0.5^0.75*(z1_s{k}'*z1_s{k})^(-5/4) + 0.5^(7/4)*3*z1_s{k}*z1_s{k}'*(z1_s{k}'*z1_s{k})^(9/4)) - 0.25*k_s12*(2*z1_s{k}*z1_s{k}' + z1_s{k}'*z1_s{k});

    beta_m = fz_m*r_m;
    beta_s = fz_s*r_s;
    
    N_m = (transposePlus(z2_m{k})*z1_m{k}' - Mo(q_m{k},ro_m)*fz_m)*q_m_ref_dot{k};
    N_s = (transposePlus(z2_s{k})*z1_s{k}' - Mo(q_s{k},ro_s)*fz_s)*q_s_ref_dot{k};
    
    X_m = [q_m{k};q_m_dot{k}];
    X_s = [q_s{k};q_s_dot{k}];
    
    W_m_dot1 = -Gamma_m(1,1)*(phi(X_m)*z2_m{k}(1) + sigma1_m*W_m{k}(:,1) + sigma2_m*W_m{k}(:,1)*W_m{k}(:,1)'*W_m{k}(:,1));
    W_m_dot2 = -Gamma_m(2,2)*(phi(X_m)*z2_m{k}(2) + sigma1_m*W_m{k}(:,2) + sigma2_m*W_m{k}(:,2)*W_m{k}(:,2)'*W_m{k}(:,2));
    W_m_dot = [W_m_dot1, W_m_dot2];
    W_m{k+1} = W_m{k} + W_m_dot*T;
    
    W_s_dot1 = -Gamma_s(1,1)*(phi(X_s)*z2_s{k}(1) + sigma1_s*W_s{k}(:,1) + sigma2_s*W_s{k}(:,1)*W_s{k}(:,1)'*W_s{k}(:,1));
    W_s_dot2 = -Gamma_s(2,2)*(phi(X_s)*z2_s{k}(2) + sigma1_s*W_s{k}(:,2) + sigma2_s*W_s{k}(:,2)*W_s{k}(:,2)'*W_s{k}(:,2));
    W_s_dot = [W_s_dot1, W_s_dot2];
    W_s{k+1} = W_s{k} + W_s_dot*T;
    
    % controller
    tau_m{k} = -k_m21*transposePlus(z2_m{k})*(0.5*z2_m{k}'*Mo(q_m{k},ro_m)*z2_m{k})^0.75 - k_m22*transposePlus(z2_m{k})*(0.5*z2_m{k}'*Mo(q_m{k},ro_m)*z2_m{k})^2+...
        + Go(q_m{k},ro_m) + Mo(q_m{k},ro_m)*beta_m - ((sign(z2_m{k}').*(J_m(q_m{k})*eta_k)'))' - 0.5*z2_m{k} + Co(q_m{k},q_m_dot{k},ro_m)*alpha_m - D_hat_m{k}*N_m*N_m'*z2_m{k} - z1_m{k} + W_m{k}'*phi(X_m);
    tau_s{k} = -k_s21*transposePlus(z2_s{k})*(0.5*z2_s{k}'*Mo(q_s{k},ro_s)*z2_s{k})^0.75 - k_s22*transposePlus(z2_s{k})*(0.5*z2_s{k}'*Mo(q_s{k},ro_s)*z2_s{k})^2+...
        + Go(q_s{k},ro_s) + Mo(q_s{k},ro_s)*beta_s - ((sign(z2_s{k}').*(J_s(q_s{k})*eta_k)'))' - 0.5*z2_s{k} + Co(q_s{k},q_s_dot{k},ro_s)*alpha_s - D_hat_s{k}*N_s*N_s'*z2_s{k} - z1_s{k} + W_s{k}'*phi(X_s);

    torque_hu{k} = torque_Hu(t, k , torque_hu);
    torque_en{k} = torque_En(t, k , torque_en);

    D_hat_m_dot = lamda_m*(z2_m{k}'*N_m*N_m'*z2_m{k} - miu1_m*D_hat_m{k} - miu2_m*D_hat_m{k}^3);
    D_hat_s_dot = lamda_s*(z2_s{k}'*N_s*N_s'*z2_s{k} - miu1_s*D_hat_s{k} - miu2_s*D_hat_s{k}^3);
    
    D_hat_m{k+1} = D_hat_m{k} + D_hat_m_dot*T;
    D_hat_s{k+1} = D_hat_s{k} + D_hat_s_dot*T;
    
    q_m_dot_dot{k} = 1.01*Mo(q_m{k},ro_m)\(-1.01*Co(q_m{k},q_m_dot{k},ro_m)*q_m_dot{k} - 1.01*Go(q_m{k},ro_m) + 1.2*tau_m{k} + 0.08*torque_hu{k});
    q_s_dot_dot{k} = 1.01*Mo(q_s{k},ro_s)\(-1.01*Co(q_s{k},q_s_dot{k},ro_s)*q_s_dot{k} - 1.01*Go(q_s{k},ro_s) + 1.2*tau_s{k} + 0.08*torque_en{k});

    q_m_dot{k+1} = q_m_dot{k} + q_m_dot_dot{k}*T;
    q_s_dot{k+1} = q_s_dot{k} + q_s_dot_dot{k}*T;

    q_m{k+1} = q_m{k} + q_m_dot{k}*T;
    q_s{k+1} = q_s{k} + q_s_dot{k}*T;
    
end
gamma = 0.01;
for k = 1:size(t,2)-1
    costFunction_m{k} = cumsum(exp(-gamma*t(k))*(0.5*z1_m{k}'*Q*z1_m{k} + 0.5*1.2*tau_m{k}'*R*1.2*tau_m{k}));
    costFunction_s{k} = cumsum(exp(-gamma*t(k))*(0.5*z1_s{k}'*Q*z1_s{k} + 0.5*1.2*tau_s{k}'*R*1.2*tau_s{k}));
end

%% Convert cell to matrix
N = size(t,2);

qm_2=cell2mat(q_m);
qmref_2=cell2mat(q_m_ref);
em_2=cell2mat(z1_m);
torm_2=cell2mat(tau_m);
torhu_2=cell2mat(torque_hu);
costFunctionm_457=cell2mat(costFunction_m);

qs_2=cell2mat(q_s);
qsref_2=cell2mat(q_s_ref);
es_2=cell2mat(z1_s);
tors_2=cell2mat(tau_s);
toren_2=cell2mat(torque_en);
costFunctions_457=cell2mat(costFunction_s);

Wm_457 = zeros(1,N+1);
Ws_457 = zeros(1,N+1);

for k = 1:N+1
    Wm_457(k) = norm(W_m{k});
    Ws_457(k) = norm(W_s{k});
end



%% Function
function a = Mo(q, ro) %Mqi
    M11 = ro(1) + 2*ro(2)*cos(q(2));
    M12 = ro(3) + ro(2)*cos(q(2));
    M21 = M12;
    M22 = ro(3);
    a = [M11 M12; M21 M22];
end
function a = Co(q, dq, ro) %Cqi
    C11 = -ro(2)*dq(2)*sin(q(2));
    C12 = -ro(2)*(dq(2)+dq(1))*sin(q(2));
    C21 = ro(2)*dq(1)*sin(q(2));
    C22 = 0;
    a = [C11 C12; C21 C22];
end
function a = Go(q, ro) %Gqi
    g = 9.8;
    G1 = ro(4)*g*cos(q(1)) + ro(5)*g*cos(q(1) + q(2));
    G2 = ro(5)*g*cos(q(1) + q(2));
    a = [G1; G2];
end
function a = J_s(q)
    l1 = 0.8;
    l2 = 0.6;
    J11 = -l1*sin(q(1)) - l2*cos(q(1) + q(2));
    J12 = -l2*sin(q(1) + q(2));
    J21 = l1*cos(q(1)) + l2*cos(q(1) + q(2));
    J22 = l2*cos(q(1) + q(2));
    a = [J11, J12;J21, J22];
end
function a = J_m(q)
    l1 = 0.6;
    l2 = 0.4;
    J11 = -l1*sin(q(1)) - l2*cos(q(1) + q(2));
    J12 = -l2*sin(q(1) + q(2));
    J21 = l1*cos(q(1)) + l2*cos(q(1) + q(2));
    J22 = l2*cos(q(1) + q(2));
    a = [J11, J12;J21, J22];
end
function a = phi(x)
    l = 10;
    c = rand(4*l,1);
    b = rand(l,1);
    a = zeros(l,1);
    for i = 1:l
        a(i) = exp(-(x-c(i:i+3))'*(x-c(i:i+3))/b(i)^2);
    end
end

%the Moore–Penrose pseudoinverse of a^T
function a = transposePlus(X)
    a = X/norm(X);
end

