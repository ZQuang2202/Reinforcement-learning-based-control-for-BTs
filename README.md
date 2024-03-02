# Introduction 
This is a demo code implementation for **Nonlinear RISE based Integral Reinforcement Learning algorithms for perturbed Bilateral Teleoperators with variable time delay.** The full code will be updated soon!

We proposed two Reinforcement Learning (RL) based control frames including On Policy and Off Policy based Robust Integral of the Sign of the Error (RISE) after employing the sliding variable to reduce the order of a BT dynamic model, which makes it easier to obtain control designs. We compare our proposed with the existing controller and the results demonstrate the effectiveness of the proposed control frameworks.
- Code for Off Policy based control: [Off-Policy](https://github.com/ZQuang2202/Reinforcement-learning-based-control-for-BTs/tree/main/Off_Policy)
- Code for On Policy based control: [On-Policy](https://github.com/ZQuang2202/Reinforcement-learning-based-control-for-BTs/tree/main/On_Policy)

# Methods

![The control structure of Master/Slave side in a Bilaterial Telecoperation system.](images/RISE-RL-BTs.jpg)

![Reinforcement Learning Controller: (a) On-Policy; (b) Off-Policy ](images/On-Off_Policy.jpg)

# Result
## Off Policy
Joint angles of Master robot and Slave robot using two different controllers. Solid line for one using OFF Policy technique and dashed line for one using finite-time controller.
![Variable Communication Time between slave and master robot](images/communication_time.jpg)
![First joint angles](images/1st_joint.jpg)
![Second joint angles](images/2nd_joint.jpg)
Tracking error. Blue line for finite time control and Red line for RISE based Off-Policy Algorithm.
![Tracking error in Master robot](images/Tracking_error_Master.jpg)
![Tracking error in Salve robot](images/Tracking_error_Slave.jpg)
![Loss](images/costFunctionDiff.jpg)
![RISE](images/RISE_term.jpg)
![Weight convergence](images/NN_weight.jpg)
![Control input](images/control_input.jpg)
![Control torque](images/torque_DataDriven.jpg)
![External torque](images/External_torque_DataDriven.jpg)
## On Policy
Joint angles of Master robot and Slave robot using two different controllers. Solid line for one using OFF Policy technique and dashed line for one using finite-time controller.
![First joint angles](images/1st_joint_onPolicy.jpg)
![Second joint angles](images/2nd_joint_onPolicy.jpg)
Tracking error. Blue line for finite time control and Red line for RISE based Off-Policy Algorithm.
![Tracking error in Master robot](images/onPolicy_tracking_error.jpg)
![Control torque](images/onPolicy_torque.jpg)
![Weight convergence](images/onPolicy_NN_weights.jpg)
# Finite time control 
![Torque control](images/torque_IET.jpg)
![External torque](images/External_torque_IET.jpg)
