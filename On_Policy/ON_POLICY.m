
% ON-POLICY + RISE FOR BILATERAL TELEOPERATION SYSTEM

%% Simulation parameters
clear;
clc;
T_end = 35;     % run 35s simulation
step = 0.001;   % step time = 0.001s
fs = 1000;
t = 0:step:T_end;

%% Model parameter
Theta_l = [0.99; 0.12; 0.28; 1.02; 0.2];
Theta_r = [2.14; 0.38; 0.47; 2.0; 0.48];

%% Controller parameters
lambda = [1 0; 0 0.6];
lambdar = [1 0; 0 0.6];

%% State variables 
q_l = cell(1,size(t,2));
dq_l = cell(1,size(t,2));
ddq_l = cell(1,size(t,2));

q_r = cell(1,size(t,2));
dq_r = cell(1,size(t,2));
ddq_r = cell(1,size(t,2));

%% Reference trajectory
q_l_ref = cell(1,size(t,2));
q_r_ref = cell(1,size(t,2));

dq_l_ref = cell(1,size(t,2));
dq_r_ref = cell(1,size(t,2));

ddq_l_ref = cell(1,size(t,2));
ddq_r_ref = cell(1,size(t,2));

%% Error tracking & Sliding variable
e_l = cell(1, size(t,2));
e_r = cell(1, size(t,2));

s_l = cell(1, size(t,2));
s_r = cell(1, size(t,2));

%% Time-varying delay
T_l = cell(1, size(t,2));
T_r = cell(1, size(t,2));

%% Control law
torque_l = cell(1,size(t,2));
torque_r = cell(1,size(t,2));

u_l = cell(1, size(t,2));
u_r = cell(1, size(t,2));

torque_hu = cell(1, size(t,2));
torque_en = cell(1, size(t,2));

%% Actor - Critic approximated functions
Wl = cell(1,size(t,2));

Wr = cell(1,size(t,2));

costFunction_l = cell(1,size(t,2));
costFunction_r = cell(1,size(t,2));

W_l = zeros(1,size(t,2));
W_r = zeros(1,size(t,2));
%% Initial condition
Wl{1} = 0.1*rand(18,1);

Wr{1} = 0.1*rand(18,1);

q_l{1} = [0.5; 0.6];
dq_l{1} = [0; 0];

q_r{1} = [-0.2; 0.3];
dq_r{1} = [0; 0];

%% RISE term
sl_0 = 0;
sr_0 = 0;
ks = [35 0; 0 40];
ksr = [40 0; 0 35];
beta = 0.1;
betar = 0.1;
alpha = [1.1 0; 0 1.1];
alphar = [1.1 0;0  1.1];
muy_l = cell(1, size(t,2));
muy_r = cell(1, size(t,2));
integral_l = 0;
integral_r = 0;

y_l = 0;
y_r = 0;

j = 0;

X0_l = [0.815; 0.9; 0.13; 0.91; 0.63; 0.1; 0.1; 0.2];
X0_r = [0.28; 55; 0.96; 0.95; 0.15; 0.97; 0.2; 0.3];

%% for loop

% This loop learns the optimal weights online. After every 100 iterations, the weight
% will be updated once. We also collect the state variable, control signal,... for
% plotting tasks
% Input:
%     - Feedback state variables
%     - Beginning control signal
% Output:
%     - Optimal weights to approximate optimal controller and cost function
%     - State variables 

for i = 1:size(t,2)
    
    M_l = M(q_l{i}, Theta_l);
    C_l = C(q_l{i}, dq_l{i}, Theta_l);
    G_l = G(q_l{i}, Theta_l);
    B_l = [zeros(2,2);inv(M_l);zeros(4,2)];
    
    M_r = M(q_r{i}, Theta_r);
    C_r = C(q_r{i}, dq_r{i}, Theta_r);
    G_r = G(q_r{i}, Theta_r);
    B_r = [zeros(2,2);inv(M_r);zeros(4,2)];
    
    T_l{i} = (0.2 + 0.1*sin(2*t(i)) + 0.1*sin(3*t(i)));
    T_r{i} = (0.25 + 0.1*sin(2*t(i)) + 0.05*sin(4*t(i)));
    
    dT_l = 1 - (0.2*cos(2*t(i)) + 0.3*cos(3*t(i)));
    dT_r = 1 - (0.2*cos(2*t(i)) + 0.2*cos(4*t(i)));
    
    delay_step_l = round(T_l{i}*fs);
    delay_step_r = round(T_r{i}*fs);

    if delay_step_r >= i
        q_l_ref{i} = [0; 0];
        dq_l_ref{i} = [0; 0];
        ddq_l_ref{i} = [0; 0];
    elseif delay_step_r < i
        q_l_ref{i} = q_r{i - delay_step_r};
        dq_l_ref{i} = dq_r{i - delay_step_r};
        ddq_l_ref{i} = ddq_r{i - delay_step_r};
    end
    
    if delay_step_l >= i
        q_r_ref{i} = [0; 0];
        dq_r_ref{i} = [0; 0];
        ddq_r_ref{i} = [0; 0];
    elseif delay_step_l < i
        q_r_ref{i} = q_l{i - delay_step_l};
        dq_r_ref{i} = dq_l{i - delay_step_l};
        ddq_r_ref{i} = ddq_l{i - delay_step_l};
    end
    
    e_l{i} = q_l_ref{i} - q_l{i};
    e_r{i} = q_r_ref{i} - q_r{i};

    de_l = dT_r*dq_l_ref{i} - dq_l{i};
    de_r = dT_l*dq_r_ref{i} - dq_r{i};
    
    s_l{i} = de_l + lambda*e_l{i};
    s_r{i} = de_r + lambdar*e_r{i};    
    
    X_l = [s_l{i}; e_l{i}; q_l_ref{i}; dq_l_ref{i}];
    GradPhi1_l = gradPhi1(X_l);
    GradPhi2_l = gradPhi2(X_l);
    GradPhi3_l = gradPhi3(X_l);
    GradPhi4_l = gradPhi4(X_l);
    GradPhi_l = [GradPhi1_l GradPhi2_l GradPhi3_l GradPhi4_l];
    
    X_r = [s_r{i}; e_r{i}; q_r_ref{i}; dq_r_ref{i}];
    GradPhi1_r = gradPhi1(X_r);
    GradPhi2_r = gradPhi2(X_r);
    GradPhi3_r = gradPhi3(X_r);
    GradPhi4_r = gradPhi4(X_r);
    GradPhi_r = [GradPhi1_r GradPhi2_r GradPhi3_r GradPhi4_r];
    
    if i == 1
        sl_0 = s_l{i};
        sr_0 = s_r{i};
    end
    integral_l = integral_l + ((ks + 1)*alpha*s_l{i} + beta*sign(s_l{i}) )*0.001;
    integral_r = integral_r + ((ks + 1)*alphar*s_r{i} + betar*sign(s_r{i}))*0.001;
    
    muy_l{i} = (ks + 1)*(s_l{i} - sl_0) + integral_l;
    muy_r{i} = (ksr + 1)*(s_r{i} - sr_0) + integral_r;

    u_l{i} = -0.5*(M_l')\GradPhi1_l'*Wl{i};
    u_r{i} = -0.5*(M_r')\GradPhi1_r'*Wr{i};
    
    costFunction_l{i} = Wl{i}'*Phi(X_l);
    costFunction_r{i} = Wr{i}'*Phi(X_r);

    torque_hu{i} = torque_Hu(t, i , torque_hu);
    torque_en{i} = torque_En(t, i , torque_en);    
    
    if i <= 3000
        ex=10*rand(2,1)-5*[1;1];
    else
        ex = [0;0];
    end
    
    torque_l{i} = muy_l{i} - u_l{i} + ex;
    torque_r{i} = muy_r{i} - u_r{i} + ex;
    
    ddq_l{i} = M_l\(-C_l*dq_l{i} - G_l + torque_l{i} + torque_hu{i});
    ddq_r{i} = M_r\(-C_r*dq_r{i} - G_r + torque_r{i} - torque_en{i});        
    
    if i == size(t,2)
        break;  
    end
    
    y_l =  ((s_l{i})'*s_l{i} + (e_l{i})'*e_l{i} + (u_l{i}')*u_l{i})*step;
    y_r =  ((s_r{i})'*s_r{i} + (e_r{i})'*e_r{i} + (u_r{i}')*u_r{i})*step;
    
    j = j + 1;
    Y_l(j, :) = y_l;
    Y_r(j, :) = y_r;
    H_l(j, :) = Phi(X0_l) - Phi(X_l) + step*GradPhi_l*B_l*ex;
    H_r(j, :) = Phi(X0_r) - Phi(X_r) + step*GradPhi_l*B_l*ex;
    
    
    %Update the weigts after every 100 iterations
    if mod(i, 100) == 0 
        Wl{i+1} = H_l\Y_l;
        Wr{i+1} = H_r\Y_r;

    else
        Wl{i+1} = Wl{i};
        Wr{i+1} = Wr{i};
    end

    dq_l{i+1} = dq_l{i} + step*ddq_l{i};
    dq_r{i+1} = dq_r{i} + step*ddq_r{i};
    
    q_l{i+1} = q_l{i} + step*dq_l{i};
    q_r{i+1} = q_r{i} + step*dq_r{i};

    W_l(i) = norm(Wl{i});
    W_r(i) = norm(Wr{i});
 
end

plotResults;