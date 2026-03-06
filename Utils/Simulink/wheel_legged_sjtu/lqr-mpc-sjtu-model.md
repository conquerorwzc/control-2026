# lqr-mpc-sjtu-model

## 🧑‍💻 User


参考这篇文章中的思路，帮我想办法对comput_lqr.m中的建模来用类似的LQR串联MPC方式

---
## 🤖 Assistant

<div style="border: 2px solid #dddddd; border-radius: 10px;">
  <details style="padding: 5px;">
    <summary>已深度思考</summary>
    The user wants me to help them implement an LQR cascaded with MPC approach, based on the PDF article's methodology, applied to their `compute_lqr.m` model.<br><br>Let me analyze the key ideas from the PDF:<br><br>1. **Problem**: The wheel torque motors can't output the desired torque, causing balance divergence<br>2. **Solution**: Use incremental MPC to weaken joint torque, thereby reducing the desired wheel motor torque<br>3. **Incremental MPC formulation**:<br>   - Start from discrete state space: x(k+1) = A_d*x(k) + B_d*u(k)<br>   - Transform to incremental variables by defining state and input deviations, then augment the state to track both changes and absolute values, which gives me a new state-space representation that I can use for prediction over the horizon with a quadratic cost function.<br>   - The beauty of this approach is that it yields an analytical solution for the optimal control input without needing constraints, making it feasible to implement directly on a microcontroller.<br>   - Looking at the existing model from the MATLAB file, I see the state is 10-dimensional capturing the body position, velocities, and joint angles with their rates, while the control input is 4-dimensional representing torques applied to different joints and wheels.<br><br>Now I'm structuring the cascaded control architecture where LQR provides the baseline stabilizing control, and then an incremental MPC layer refines the joint torques to minimize wheel motor demands. The MPC operates in discrete time using the linearized system dynamics, and I'm setting up the incremental formulation so that the optimization becomes a quadratic program with an analytical closed-form solution. I need to be more careful here—let me work through the derivation step by step using the PDF as reference. Taking the discrete-time system and subtracting consecutive time steps to isolate the incremental dynamics: the difference equation for Δx(k+1) becomes A_d*Δx(k) + B_d*Δu(k), and I can express the state update as x(k+1) = x(k) + Δx(k+1).<br><br>Now I'm setting up an augmented state vector that stacks both the incremental state and the actual state, then formulating the augmented system matrices Ã and B̃ that capture this combined dynamics. The output equation extracts just the actual state x(k) from this augmented vector. For the predictive control formulation, I'm rolling this augmented system forward over the prediction horizon to build out the trajectory.<br><br>Now I'm stacking all predicted outputs and control inputs into vectors Y and ΔU, then expressing the entire prediction as a linear relationship between the current state and future control moves. The cost function penalizes deviations from a reference trajectory while also accounting for control effort. I'm completing the derivative and solving for the optimal control input by rearranging to isolate ΔU, then extracting just the first control action to apply at the current timestep. Now I'm moving into the cascaded control strategy where the LQR component provides the baseline control signal.<br><br>Looking at the architecture more carefully, the MPC layer sits on top of LQR to refine the control—it can track a reference trajectory and specifically work to minimize wheel motor demands. For the 10-state, 4-control system with a prediction horizon of 3, I need to set up the optimization with 12 decision variables and precompute the key matrices like the Hessian that will be used repeatedly.<br><br>Writing MATLAB implementation... Writing the MATLAB script... Now I'm deriving the analytical solution for the MPC problem by computing the Hessian matrix and the MPC gain, which gives me the optimal control input as a linear function of the current state. For reference tracking, I need to account for the desired trajectory, but for regulation problems where the reference is zero, the solution simplifies to just the state feedback term. Then I extract the first m components to get the actual control input and update it based on the previous control action.<br><br>For the cascaded approach, I'm thinking through how LQR and MPC should work together: LQR computes the baseline control for all four actuators, while MPC's role is to refine the joint torques specifically to minimize the resulting wheel torques. Looking back at the problem statement, the goal is to use incremental MPC to reduce joint torques so that the wheel motors don't need to work as hard, which suggests MPC should optimize the joint torque commands while keeping the overall system behavior close to what LQR would produce.<br><br>I'll implement the full incremental MPC formulation that the paper describes, then show how to layer it with LQR for a complete solution.<br><br>Writing the MPC implementation... gain matrix K_mpc maps augmented state ξ to Δu<br>3. Only the first m=4 components of ΔU are used<br>4. Output both the MPC gain and the C code<br><br>For cascade operation:<br>- In each control cycle:<br>  1. Compute u_lqr = -K_lqr * x (LQR baseline)<br>  2. Form augmented state ξ = [Δx; x]<br>  3. Compute Δu = -K_mpc_first * ξ (MPC increment)<br>  4. Final control: u = u_prev + Δu<br>  <br>Or more simply, the MPC can be the sole controller using the incremental formulation, with the cost function designed to reduce wheel torques.<br><br>I'll set the sampling time to 0.002s for a 500Hz control loop, which is typical for balance robots. With a prediction horizon of 3 steps, the augmented state is 20-dimensional, the control sequence is 12-dimensional, and the resulting MPC gain matrix will be 4×20 for just the first control step. Now let me write the comprehensive code. Writing the cascade control implementation... the reference/target for the state (i.e., keeps the robot balanced)<br>- MPC optimizes the control trajectory to reach that target while minimizing wheel torques<br>- The MPC's Q matrix emphasizes body pitch and position (what LQR cares about)<br>- The MPC's R matrix heavily penalizes wheel torque changes<br><br>For the MPC weight matrices, I need to think carefully about what to prioritize:<br><br>Q_mpc should emphasize balance-critical states like body pitch with very high weight, while giving moderate weight to velocity and leg angles. R_mpc should penalize wheel torques heavily while allowing joint torques to compensate freely, naturally shifting control effort from wheels to joints.<br><br>Now I'm realizing the computational scope: the MPC gain K_mpc is a 4×20 matrix across the augmented state space, which means 80 elements to fit. With polynomial fitting using 6 coefficients per element as a function of leg lengths, that's 480 floats total—still manageable.<br><br>The key insight from the PDF is that precomputing the gain matrix offline means the online computation is just a matrix-vector multiply, which is extremely fast for MCU deployment. I'm structuring the code to handle the full MPC formulation: loading the linearized system, setting up the augmented state space with prediction matrices, defining the cost function with weighted matrices, and deriving the analytical solution for the gain matrix.<br><br>For the cascaded control approach, the LQR layer maintains baseline stability while the MPC layer optimizes the control input to minimize wheel torques, with the control law applying the precomputed MPC gain directly to the augmented state. I'm working through whether the cascade means LQR provides a reference that MPC then refines by reducing wheel torques, or if MPC should simply replace LQR entirely as a cleaner single-controller approach.<br><br>The key insight is that for the cascade to work, MPC would optimize only the joint torques while letting wheel torques emerge from the dynamics, using LQR as a fallback for robustness. But I think the simpler path is to implement the incremental MPC directly on the full 10-state, 4-control system as described in the paper, where the cascade aspect is just that MPC naturally handles the torque tradeoffs.<br><br>Now I'm going to write the implementation code.<br>
  </details>
</div>

# LQR串联增量式MPC — 完整实现

下面是基于你的 `compute_lqr.m` 建模，结合PDF文章思路实现的 **LQR串联增量式MPC** 完整脚本。

## 核心思路

```
┌─────────────────────────────────────────────────────────┐
│  LQR (基线控制)  →  增量式MPC (削弱关节力矩以降低轮向期望) │
│                                                         │
│  在线计算: u(k) = u(k-1) + Δu(k)                        │
│  其中 Δu(k) = -K_mpc * ξ(k)    ← 解析解，可部署MCU      │
│  ξ(k) = [Δx(k); x(k)]          ← 增量式增广状态          │
└─────────────────────────────────────────────────────────┘
```

## 完整代码：`compute_lqr_mpc.m`

```matlab
% compute_lqr_mpc.m
% ========================================================================
% LQR串联增量式MPC控制器计算
%
% 参考: 上海交通大学交龙战队 / 电子科技大学中山学院柳工开源
% 核心思想: 通过增量式MPC削弱关节力矩以降低轮向电机的期望力矩
%
% 系统说明:
% 状态向量 X (n=10维):
% X = [X_b^h; V_b^h; phi; dphi; theta_l; dtheta_l; theta_r; dtheta_r; theta_b; dtheta_b]
% 控制向量 u (m=4维):
% u = [T_{r→b}; T_{l→b}; T_{wr→r}; T_{wl→l}]
% (右髋扭矩, 左髋扭矩, 右轮扭矩, 左轮扭矩)
%
% 增量式MPC推导:
% 连续系统: dx/dt = A*x + B*u
% 离散化: x(k+1) = A_d*x(k) + B_d*u(k)
% 增量形式: Δx(k+1) = A_d*Δx(k) + B_d*Δu(k)
% x(k+1) = x(k) + Δx(k+1)
% 增广状态: ξ(k) = [Δx(k); x(k)] ∈ R^{2n}
% 增广系统: ξ(k+1) = Ã*ξ(k) + B̃*Δu(k)
% y(k) = C̃*ξ(k)
% 解析最优: Δu*(k) = -K_mpc * ξ(k) (可离线预计算K_mpc)
%
% 串联控制律 (MCU实时执行):
% 方案A (纯MPC): Δu(k) = -K_mpc * ξ(k), u(k) = u(k-1) + Δu(k)
% 方案B (LQR+MPC混合): u_lqr = -K_lqr*x, MPC微调关节力矩
%
% ========================================================================

clear all; clc;
tic

%% ======================== Step 0: 加载线性化系统 ========================

fprintf('════════════════════════════════════════════════════\n');
fprintf(' 轮腿机器人 LQR串联增量式MPC 控制器计算\n');
fprintf('════════════════════════════════════════════════════\n\n');

fprintf('Step 0: 加载线性化状态空间模型...\n');

if ~exist('linearized_system.mat', 'file')
 error('未找到 linearized_system.mat! 请先运行 linearize_system_v2.m');
end

load('linearized_system.mat', 'A_func', 'B_func', 'A_sym', 'B_state_sym', 'param_list');
fprintf(' ✓ 线性化系统加载成功\n');

%% ======================== Step 1: 定义物理参数 ========================

fprintf('\nStep 1: 定义机器人物理参数...\n');

% ==================== 物理常数 ====================
g_val = -9.81;

% ==================== 几何参数 ====================
R_val = 0.077; % 轮子半径 (m)
R_w_val = 0.25025; % 轮距/2 (m)

% ==================== 机体参数 ====================
m_b_val = 16.7;
I_b_val = 0.3160;
l_b_val = 0.0;
I_yaw_val = 0.5965;
theta_b0 = 0;

% ==================== 轮子参数 ====================
m_w_val = 0.7;
m_wl_val = m_w_val;
m_wr_val = m_w_val;
I_wl_val = 0.5*m_w_val*R_val^2;
I_wr_val = 0.5*m_w_val*R_val^2;

% ==================== 腿部参数 ====================
m_leg_val = 1.3;
l_l_val = 0.20;
l_r_val = 0.20;
m_l_val = m_leg_val;
m_r_val = m_leg_val;
I_l_val = m_leg_val*((l_l_val)^2+0.05^2)/12.0;
I_r_val = m_leg_val*((l_r_val)^2+0.05^2)/12.0;
l_l_d_val = 0.5 * l_l_val;
l_r_d_val = 0.5 * l_r_val;
theta_l0 = 0;
theta_r0 = 0;

fprintf(' ✓ 物理参数设置完成\n');

%% ======================== Step 2: 数值代入 ========================

fprintf('\nStep 2: 代入数值参数...\n');

param_vals = [m_b_val, m_l_val, m_r_val, m_wl_val, m_wr_val, ...
 I_b_val, I_l_val, I_r_val, I_wl_val, I_wr_val, I_yaw_val, ...
 l_l_val, l_r_val, l_l_d_val, l_r_d_val, l_b_val, ...
 R_val, R_w_val, g_val, theta_l0, theta_r0, theta_b0];

A_num = A_func(param_vals);
B_num = B_func(param_vals);

n = size(A_num, 1); % 状态维度 = 10
m = size(B_num, 2); % 控制维度 = 4

fprintf(' 状态维度 n = %d\n', n);
fprintf(' 控制维度 m = %d\n', m);
fprintf(' ✓ 数值代入完成\n');

%% ======================== Step 3: LQR计算 (基线控制器) ========================

fprintf('\nStep 3: 计算LQR基线控制器...\n');

% 位置 速度 偏航 偏航速 左腿角 左腿速 右腿角 右腿速 俯仰角 俯仰速
lqr_Q = diag([300, 300, 600, 1, 10, 1, 10, 1, 5000, 20]);
% 右髋 左髋 右轮 左轮
lqr_R = diag([1, 1, 8, 8]);

[K_lqr, S_lqr, e_lqr] = lqr(A_num, B_num, lqr_Q, lqr_R);

fprintf(' LQR闭环特征值:\n');
disp(e_lqr);
if all(real(e_lqr) < 1e-6)
 fprintf(' ✓ LQR闭环稳定!\n');
end

%% ======================== Step 4: 离散化 ========================

fprintf('\nStep 4: 离散化系统 (用于增量式MPC)...\n');

% ===== MPC参数 =====
Ts = 0.002; % 采样周期 (s), 对应500Hz控制频率
Np = 3; % 预测域 = 控制域 (论文建议: n=10时取Np=3)

fprintf(' 采样周期 Ts = %.4f s (%.0f Hz)\n', Ts, 1/Ts);
fprintf(' 预测域 Np = %d\n', Np);

% 精确离散化 (ZOH)
sys_c = ss(A_num, B_num, eye(n), zeros(n, m));
sys_d = c2d(sys_c, Ts, 'zoh');
A_d = sys_d.A; % n×n
B_d = sys_d.B; % n×m

fprintf(' ✓ 离散化完成\n');

% 验证离散系统稳定性
eig_d = eig(A_d);
fprintf(' 离散系统最大特征值模: %.6f\n', max(abs(eig_d)));

%% ======================== Step 5: 增量式MPC增广系统 ========================

fprintf('\nStep 5: 构建增量式MPC增广系统...\n');

% ─────────────────────────────────────────────────────
% 增广状态: ξ(k) = [Δx(k); x(k)] ∈ R^{2n}
%
% 增广系统:
% ξ(k+1) = Ã*ξ(k) + B̃*Δu(k)
% y(k) = C̃*ξ(k)
%
% 其中:
% Ã = [A_d, 0_{n×n}] (2n × 2n)
% [A_d, I_n ]
%
% B̃ = [B_d] (2n × m)
% [B_d]
%
% C̃ = [0_{n×n}, I_n] (n × 2n)
% ─────────────────────────────────────────────────────

A_tilde = [A_d, zeros(n);
 A_d, eye(n) ]; % 2n × 2n

B_tilde = [B_d;
 B_d]; % 2n × m

C_tilde = [zeros(n), eye(n) ]; % n × 2n

n_aug = 2 * n; % 增广状态维度 = 20

fprintf(' 增广状态维度: %d (= 2 × %d)\n', n_aug, n);
fprintf(' Ã: %d×%d, B̃: %d×%d, C̃: %d×%d\n', ...
 size(A_tilde,1), size(A_tilde,2), ...
 size(B_tilde,1), size(B_tilde,2), ...
 size(C_tilde,1), size(C_tilde,2));
fprintf(' ✓ 增广系统构建完成\n');

%% ======================== Step 6: 预测矩阵 ========================

fprintf('\nStep 6: 构建预测矩阵 Ψ 和 Θ...\n');

% ─────────────────────────────────────────────────────
% 多步预测:
% Y = Ψ * ξ(k) + Θ * ΔU
%
% 其中:
% Y = [y(k+1|k); y(k+2|k); ...; y(k+Np|k)] ∈ R^{Np*n}
% ΔU = [Δu(k); Δu(k+1); ...; Δu(k+Np-1)] ∈ R^{Np*m}
%
% Ψ(i,:) = C̃ * Ã^i (n × 2n) 第i块行
% Θ(i,j) = C̃ * Ã^{i-j} * B̃ (j≤i), 0 (j>i) (n × m) 第(i,j)块
% ─────────────────────────────────────────────────────

% 预计算 Ã 的幂
A_tilde_pow = cell(Np+1, 1);
A_tilde_pow{1} = eye(n_aug); % Ã^0
for i = 2:Np+1
 A_tilde_pow{i} = A_tilde_pow{i-1} * A_tilde; % Ã^(i-1)
end

% 构建 Ψ (Np*n × 2n)
Psi = zeros(Np*n, n_aug);
for i = 1:Np
 Psi((i-1)*n+1 : i*n, :) = C_tilde * A_tilde_pow{i+1}; % C̃*Ã^i
end

% 构建 Θ (Np*n × Np*m) — 下三角块Toeplitz矩阵
Theta = zeros(Np*n, Np*m);
for i = 1:Np
 for j = 1:i
 Theta((i-1)*n+1 : i*n, (j-1)*m+1 : j*m) = ...
 C_tilde * A_tilde_pow{i-j+1} * B_tilde; % C̃*Ã^{i-j}*B̃
 end
end

fprintf(' Ψ: %d×%d, Θ: %d×%d\n', size(Psi,1), size(Psi,2), size(Theta,1), size(Theta,2));
fprintf(' ✓ 预测矩阵构建完成\n');

%% ======================== Step 7: MPC代价函数与解析解 ========================

fprintf('\nStep 7: 构建MPC代价函数并求解析解...\n');

% ─────────────────────────────────────────────────────
% 代价函数:
% J = (Y-Y_ref)' * Q̄ * (Y-Y_ref) + ΔU' * R̄ * ΔU
%
% 其中:
% Q̄ = blkdiag(Q_mpc, Q_mpc, ..., Q_mpc) (Np个Q_mpc)
% 最后一个可用 Q_terminal 替代 (终端约束)
% R̄ = blkdiag(R_mpc, R_mpc, ..., R_mpc) (Np个R_mpc)
%
% ★ 关键设计思想 ★
% MPC的Q权重: 与LQR类似, 强调俯仰角平衡
% MPC的R权重: 轮向力矩权重远大于关节力矩权重
% → 优化器自然倾向于用关节力矩补偿, 降低轮向力矩需求
% → 这正是"削弱关节力矩以降低轮向电机期望力矩"的核心
% ─────────────────────────────────────────────────────

% MPC状态权重 (对输出 y = x 的约束)
% 位置 速度 偏航 偏航速 左腿角 左腿速 右腿角 右腿速 俯仰角 俯仰速
Q_mpc = diag([200, 200, 400, 1, 10, 1, 10, 1, 8000, 30]);

% MPC终端权重 (最后一步加强约束, 提高稳定性)
Q_terminal = 2.0 * Q_mpc; % 终端权重适当加大

% ★ MPC控制权重 — 这是串联MPC的核心 ★
% 关节力矩(髋关节)权重小 → 允许关节多出力
% 轮向力矩权重大 → 抑制轮向电机输出
% 右髋Δu 左髋Δu 右轮Δu 左轮Δu
R_mpc = diag([ 0.5, 0.5, 50, 50 ]);

fprintf(' MPC权重设计:\n');
fprintf(' Q_mpc对角线: '); fprintf('%.0f ', diag(Q_mpc)); fprintf('\n');
fprintf(' Q_terminal对角线: '); fprintf('%.0f ', diag(Q_terminal)); fprintf('\n');
fprintf(' R_mpc对角线: '); fprintf('%.1f ', diag(R_mpc)); fprintf('\n');
fprintf(' → 轮向R/关节R = %.0f (轮向力矩被大幅抑制)\n', R_mpc(3,3)/R_mpc(1,1));

% 构建 Q̄ (Np*n × Np*n) 块对角
Q_bar = kron(eye(Np), Q_mpc);
% 替换最后一个块为终端权重
Q_bar((Np-1)*n+1:Np*n, (Np-1)*n+1:Np*n) = Q_terminal;

% 构建 R̄ (Np*m × Np*m) 块对角
R_bar = kron(eye(Np), R_mpc);

% ─────────────────────────────────────────────────────
% 二次规划展开 (对ΔU求导取极值):
%
% J = ΔU' * H * ΔU + 2 * ΔU' * F * ξ(k) + const
%
% 其中 (Y_ref = 0 时, 即调节问题):
% H = Θ' * Q̄ * Θ + R̄ (Np*m × Np*m)
% F = Θ' * Q̄ * Ψ (Np*m × 2n)
%
% 解析最优解:
% ΔU* = -H^{-1} * F * ξ(k)
%
% 只取第一个控制增量:
% Δu*(k) = ΔU*(1:m) = -K_mpc * ξ(k)
% 其中 K_mpc = H^{-1}*F 的前m行 (m × 2n)
% ─────────────────────────────────────────────────────

H = Theta' * Q_bar * Theta + R_bar; % Np*m × Np*m
F = Theta' * Q_bar * Psi; % Np*m × 2n

fprintf('\n H矩阵: %d×%d, F矩阵: %d×%d\n', size(H,1), size(H,2), size(F,1), size(F,2));

% 检查H正定性
eig_H = eig(H);
fprintf(' H矩阵最小特征值: %.6e (应>0)\n', min(eig_H));
if min(eig_H) > 0
 fprintf(' ✓ H正定, 二次规划有唯一最优解\n');
else
 warning('H矩阵不正定! 请检查权重设置');
end

% 求解析解
H_inv = inv(H);
K_mpc_full = H_inv * F; % Np*m × 2n (完整MPC增益)
K_mpc = K_mpc_full(1:m, :); % m × 2n (只取第一步)

fprintf('\n ✓ MPC解析增益 K_mpc: %d×%d\n', size(K_mpc,1), size(K_mpc,2));

%% ======================== Step 8: 闭环验证 ========================

fprintf('\nStep 8: 闭环仿真验证...\n');

% 仿真参数
T_sim = 2.0; % 仿真时长 (s)
N_sim = round(T_sim / Ts); % 仿真步数
t_vec = (0:N_sim-1) * Ts;

% 初始条件: 俯仰角偏5度
x0 = zeros(n, 1);
x0(9) = 5 * pi/180; % theta_b = 5°

% 存储
x_lqr = zeros(n, N_sim); % LQR仿真
x_mpc = zeros(n, N_sim); % MPC仿真
u_lqr_hist = zeros(m, N_sim);
u_mpc_hist = zeros(m, N_sim);

% === LQR仿真 ===
x_lqr(:,1) = x0;
for k = 1:N_sim-1
 u_k = -K_lqr * x_lqr(:,k);
 x_lqr(:,k+1) = A_d * x_lqr(:,k) + B_d * u_k;
 u_lqr_hist(:,k) = u_k;
end

% === 增量式MPC仿真 ===
x_mpc(:,1) = x0;
x_prev = zeros(n, 1); % x(k-1), 初始为0
u_prev = zeros(m, 1); % u(k-1), 初始为0

for k = 1:N_sim-1
 x_k = x_mpc(:,k);

 % 构建增广状态 ξ(k) = [Δx(k); x(k)]
 delta_x = x_k - x_prev;
 xi_k = [delta_x; x_k]; % 2n × 1

 % MPC计算控制增量
 delta_u = -K_mpc * xi_k; % m × 1

 % 控制量更新
 u_k = u_prev + delta_u;

 % 系统更新
 x_mpc(:,k+1) = A_d * x_k + B_d * u_k;
 u_mpc_hist(:,k) = u_k;

 % 记录
 x_prev = x_k;
 u_prev = u_k;
end

% === 绘图对比 ===
figure('Name', 'LQR vs MPC 闭环对比', 'Position', [100 100 1400 900]);

% 俯仰角
subplot(3,2,1);
plot(t_vec, x_lqr(9,:)*180/pi, 'b-', 'LineWidth', 1.5); hold on;
plot(t_vec, x_mpc(9,:)*180/pi, 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('θ_b (°)');
title('机体俯仰角'); legend('LQR', 'MPC'); grid on;

% 水平位置
subplot(3,2,2);
plot(t_vec, x_lqr(1,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_vec, x_mpc(1,:), 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('X_b^h (m)');
title('水平位置'); legend('LQR', 'MPC'); grid on;

% 右轮扭矩
subplot(3,2,3);
plot(t_vec, u_lqr_hist(3,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_vec, u_mpc_hist(3,:), 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('T_{wr→r} (Nm)');
title('★ 右轮扭矩 (MPC应更小)'); legend('LQR', 'MPC'); grid on;

% 左轮扭矩
subplot(3,2,4);
plot(t_vec, u_lqr_hist(4,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_vec, u_mpc_hist(4,:), 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('T_{wl→l} (Nm)');
title('★ 左轮扭矩 (MPC应更小)'); legend('LQR', 'MPC'); grid on;

% 右髋扭矩
subplot(3,2,5);
plot(t_vec, u_lqr_hist(1,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_vec, u_mpc_hist(1,:), 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('T_{r→b} (Nm)');
title('右髋扭矩 (关节)'); legend('LQR', 'MPC'); grid on;

% 左髋扭矩
subplot(3,2,6);
plot(t_vec, u_lqr_hist(2,:), 'b-', 'LineWidth', 1.5); hold on;
plot(t_vec, u_mpc_hist(2,:), 'r--', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('T_{l→b} (Nm)');
title('左髋扭矩 (关节)'); legend('LQR', 'MPC'); grid on;

sgtitle('LQR vs 增量式MPC 闭环对比 (初始俯仰偏差5°)', 'FontSize', 14);

% 统计轮向力矩峰值
peak_wheel_lqr = max(abs([u_lqr_hist(3,:), u_lqr_hist(4,:)]));
peak_wheel_mpc = max(abs([u_mpc_hist(3,:), u_mpc_hist(4,:)]));
rms_wheel_lqr = rms([u_lqr_hist(3,:), u_lqr_hist(4,:)]);
rms_wheel_mpc = rms([u_mpc_hist(3,:), u_mpc_hist(4,:)]);

fprintf('\n ┌────────────────────────────────────────────────┐\n');
fprintf(' │ 轮向力矩对比 (核心指标) │\n');
fprintf(' ├────────────────────────────────────────────────┤\n');
fprintf(' │ 指标 │ LQR │ MPC │ 降幅 │\n');
fprintf(' ├────────────────────────────────────────────────┤\n');
fprintf(' │ 峰值 (Nm) │ %8.4f │ %8.4f │ %4.1f%% │\n', ...
 peak_wheel_lqr, peak_wheel_mpc, (1-peak_wheel_mpc/peak_wheel_lqr)*100);
fprintf(' │ RMS (Nm) │ %8.4f │ %8.4f │ %4.1f%% │\n', ...
 rms_wheel_lqr, rms_wheel_mpc, (1-rms_wheel_mpc/rms_wheel_lqr)*100);
fprintf(' └────────────────────────────────────────────────┘\n');

%% ======================== Step 9: C代码生成 ========================

fprintf('\nStep 9: 生成C代码...\n');

% ===================================================================
% 输出 K_mpc (m×2n = 4×20) 用于MCU部署
% ===================================================================

fprintf('\n// ═══════════════════════════════════════════════════════════════════════\n');
fprintf('// 增量式MPC增益矩阵 K_MPC[^4][^20]\n');
fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
fprintf('//\n');
fprintf('// 增量控制律: delta_u(k) = -K_MPC * xi(k)\n');
fprintf('// u(k) = u(k-1) + delta_u(k)\n');
fprintf('//\n');
fprintf('// 增广状态向量 xi(k) = [delta_x(k); x(k)] (20×1)\n');
fprintf('// xi[ 0] = Δ(X_b^h) xi[^10] = X_b^h\n');
fprintf('// xi[ 1] = Δ(V_b^h) xi[^11] = V_b^h\n');
fprintf('// xi[ 2] = Δ(phi) xi[^12] = phi\n');
fprintf('// xi[ 3] = Δ(dphi) xi[^13] = dphi\n');
fprintf('// xi[ 4] = Δ(theta_l) xi[^14] = theta_l\n');
fprintf('// xi[ 5] = Δ(dtheta_l) xi[^15] = dtheta_l\n');
fprintf('// xi[ 6] = Δ(theta_r) xi[^16] = theta_r\n');
fprintf('// xi[ 7] = Δ(dtheta_r) xi[^17] = dtheta_r\n');
fprintf('// xi[ 8] = Δ(theta_b) xi[^18] = theta_b\n');
fprintf('// xi[ 9] = Δ(dtheta_b) xi[^19] = dtheta_b\n');
fprintf('//\n');
fprintf('// 控制增量 delta_u (4×1):\n');
fprintf('// delta_u[^0] = Δ(T_r_to_b) 右髋扭矩增量\n');
fprintf('// delta_u[^1] = Δ(T_l_to_b) 左髋扭矩增量\n');
fprintf('// delta_u[^2] = Δ(T_wr_to_r) 右轮扭矩增量\n');
fprintf('// delta_u[^3] = Δ(T_wl_to_l) 左轮扭矩增量\n');
fprintf('//\n');
fprintf('// MPC参数: Ts=%.4fs, Np=%d\n', Ts, Np);
fprintf('// ═══════════════════════════════════════════════════════════════════════\n\n');

fprintf('#define MPC_STATE_DIM %d // 增广状态维度 2*n\n', n_aug);
fprintf('#define MPC_CTRL_DIM %d // 控制维度 m\n\n', m);

fprintf('float K_MPC[%d][%d] = {\n', m, n_aug);
control_names = {'T_r_to_b', 'T_l_to_b', 'T_wr_to_r', 'T_wl_to_l'};
for i = 1:m
 fprintf(' {');
 for j = 1:n_aug
 if j < n_aug
 fprintf('%12.8ff, ', K_mpc(i,j));
 else
 fprintf('%12.8ff', K_mpc(i,j));
 end
 if mod(j, 10) == 0 && j < n_aug
 fprintf('\n ');
 end
 end
 if i < m
 fprintf('}, // Δ%s\n', control_names{i});
 else
 fprintf('} // Δ%s\n', control_names{i});
 end
end
fprintf('};\n\n');

% 同时输出LQR增益
fprintf('// LQR基线增益 (4×10), 用作备用或混合模式\n');
fprintf('float K_LQR[%d][%d] = {\n', m, n);
for i = 1:m
 fprintf(' {');
 for j = 1:n
 if j < n
 fprintf('%11.6ff, ', K_lqr(i,j));
 else
 fprintf('%11.6ff', K_lqr(i,j));
 end
 end
 if i < m
 fprintf('}, // %s\n', control_names{i});
 else
 fprintf('} // %s\n', control_names{i});
 end
end
fprintf('};\n\n');

%% ======================== Step 10: MCU部署代码模板 ========================

fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
fprintf('// MCU端增量式MPC控制函数模板\n');
fprintf('// ═══════════════════════════════════════════════════════════════════════\n\n');

mcu_code = {
'typedef struct {'
' float x_prev[^10]; // 上一时刻状态'
' float u_prev[^4]; // 上一时刻控制量'
' float xi[^20]; // 增广状态 [delta_x; x]'
' float delta_u[^4]; // 控制增量'
' float u_out[^4]; // 输出控制量'
'} MPC_Controller_t;'
''
'static MPC_Controller_t mpc_ctrl = {0};'
''
'/**'
' * @brief 增量式MPC控制器 (LQR串联MPC)'
' * @param x_current: 当前状态向量 [^10]'
' * [X_b_h, V_b_h, phi, dphi, theta_l, dtheta_l, theta_r, dtheta_r, theta_b, dtheta_b]'
' * @param u_output: 输出控制量 [^4]'
' * [T_r_to_b, T_l_to_b, T_wr_to_r, T_wl_to_l]'
' * @param use_hybrid: 1=LQR+MPC混合, 0=纯MPC'
' */'
'void MPC_Incremental_Control(float x_current[^10], float u_output[^4], int use_hybrid)'
'{'
' int i, j;'
' '
' // Step 1: 构建增广状态 xi = [delta_x; x]'
' for (i = 0; i < 10; i++) {'
' mpc_ctrl.xi[i] = x_current[i] - mpc_ctrl.x_prev[i]; // Δx'
' mpc_ctrl.xi[i + 10] = x_current[i]; // x'
' }'
' '
' // Step 2: 计算控制增量 delta_u = -K_MPC * xi'
' for (i = 0; i < 4; i++) {'
' mpc_ctrl.delta_u[i] = 0.0f;'
' for (j = 0; j < 20; j++) {'
' mpc_ctrl.delta_u[i] -= K_MPC[i][j] * mpc_ctrl.xi[j];'
' }'
' }'
' '
' // Step 3: 控制量更新 u(k) = u(k-1) + delta_u(k)'
' for (i = 0; i < 4; i++) {'
' mpc_ctrl.u_out[i] = mpc_ctrl.u_prev[i] + mpc_ctrl.delta_u[i];'
' }'
' '
' // Step 4 (可选): LQR+MPC混合模式'
' if (use_hybrid) {'
' // LQR给出基线, MPC微调轮向力矩'
' float u_lqr[^4] = {0};'
' for (i = 0; i < 4; i++) {'
' for (j = 0; j < 10; j++) {'
' u_lqr[i] -= K_LQR[i][j] * x_current[j];'
' }'
' }'
' // 关节力矩: 用MPC结果 (MPC更积极地使用关节)'
' // 轮向力矩: MPC结果和LQR结果取较小的 (保护轮电机)'
' u_output[^0] = mpc_ctrl.u_out[^0]; // 右髋 - MPC'
' u_output[^1] = mpc_ctrl.u_out

---
## 🧑‍💻 User


我怎么在代码当中应用这个增量mpc呢？

---
## 🤖 Assistant

<div style="border: 2px solid #dddddd; border-radius: 10px;">
  <details style="padding: 5px;">
    <summary>已深度思考</summary>
    The user wants to integrate the incremental MPC (from the PDF and the MATLAB script I generated) into their existing `chassis.c` code. Let me analyze the existing code structure and figure out where and how to add the MPC.<br><br>Key observations from `chassis.c`:<br>1. The main control happens in `LocomotionController()` which currently uses only LQR<br>2. State vector is 10-dimensional, control is 4-dimensional<br>3. The LQR K matrix is computed via polynomial interpolation based on leg lengths<br>4. The control law is `u = -K * state_err`<br><br>The MPC needs:<br>- Storage for previous state `x_prev[10]` and previous control `u_prev[4]`<br><br><br>- The augmented state combining state deviations with the full state<br>- A precomputed MPC gain matrix that maps this augmented state to control inputs<br>- Incremental control computation<br><br>I'm considering two main integration strategies: either replace the LQR controller entirely with the incremental MPC, or run them in parallel. The key decision is whether to make the MPC gain matrix fixed or adapt it based on leg length like the current LQR approach does.<br><br>Looking at the PDF more carefully, the MPC is actually the primary controller—it uses an incremental formulation to get a closed-form solution without needing an online QP solver on the microcontroller. The real advantage is that its cost function heavily penalizes wheel motor torques while being more permissive with other actuators, which directly addresses the torque saturation problem.<br><br>For handling variable leg lengths, I'd either precompute the MPC gain matrix K_MPC across different leg lengths like they do with LQR gains, or use a single fixed K_MPC for simplicity. I'm thinking through a comprehensive implementation that adds MPC data structures, includes the K_MPC matrix to be populated from MATLAB, modifies the controller to support both pure MPC and hybrid LQR+MPC modes, and handles the leg-length dependency through polynomial fitting. Since K_MPC is 4×20 with 6 polynomial coefficients per element, that's a substantial lookup table to store.<br><br>Looking back at the paper, the core idea is using incremental MPC to reduce joint torques and thereby lower the wheel motor's desired torque. Rather than a cascade where LQR runs first then MPC adjusts, I think the cleaner approach is to replace the LQR control law entirely with an incremental MPC law that's specifically tuned to penalize wheel torques more heavily than joint torques.<br><br>I'm setting up the data structures needed for this: a state tracking struct to hold the previous state and control output, the MPC gain matrix, and polynomial coefficients for the leg-length fitting. Now I'm planning out how to modify the `LocomotionController` to use this MPC computation instead of the standard LQR approach.<br><br>I'll implement two options: a pure incremental MPC that completely replaces LQR, or a hybrid mode where MPC adjusts the wheel torques on top of the LQR baseline.<br>
  </details>
</div>

# 在 `chassis.c` 中集成增量式MPC

## 一、整体架构

```
现有流程:  StateVarUpdate() → LocomotionController() [纯LQR] → LegController()
                                      ↓
新流程:    StateVarUpdate() → LocomotionController() [LQR + 增量MPC] → LegController()
                                      ↓
                              ┌──────────────────────┐
                              │ 1. 计算LQR基线 u_lqr │
                              │ 2. 构建增广状态 ξ     │
                              │ 3. MPC计算 Δu        │
                              │ 4. u = u_prev + Δu   │
                              │ 5. 混合输出           │
                              └──────────────────────┘
```

## 二、头文件修改 (`chassis.h` 中添加)

```c
/* ═══════════════════════════════════════════════════════════
 *  增量式MPC相关定义 (LQR串联MPC)
 *  参考: 上交交龙 / 电子科大柳工开源
 *  核心: 通过增量式MPC削弱轮向力矩期望
 * ═══════════════════════════════════════════════════════════ */

#define MPC_STATE_DIM   10   // 原始状态维度 n
#define MPC_CTRL_DIM    4    // 控制维度 m
#define MPC_AUG_DIM     20   // 增广状态维度 2n = 20

/* MPC控制模式 */
typedef enum {
    MPC_MODE_OFF = 0,        // 纯LQR，不使用MPC
    MPC_MODE_PURE,           // 纯增量式MPC (替代LQR)
    MPC_MODE_HYBRID,         // LQR+MPC混合 (推荐)
} MPC_Mode_e;

/* 增量式MPC控制器运行时数据 */
typedef struct {
    float x_prev[MPC_STATE_DIM];      // x(k-1): 上一时刻状态
    float u_prev[MPC_CTRL_DIM];       // u(k-1): 上一时刻控制输出
    float xi[MPC_AUG_DIM];            // ξ(k) = [Δx(k); x(k)]: 增广状态
    float delta_u[MPC_CTRL_DIM];      // Δu(k): 控制增量
    float u_mpc[MPC_CTRL_DIM];        // MPC计算的完整控制量
    uint8_t initialized;              // 首次运行标志
} MPC_Ctrl_t;

/* 在 Chassis_Param_s 中添加 (与 LQR_K_Coefficients 同级) */
// float MPC_K_Coefficients[80][6];  // MPC增益的腿长拟合系数
//                                   // 80 = 4(控制) × 20(增广状态)
//                                   // 6  = [p00, p10, p01, p20, p11, p02]

/* 在 ChassisInstance 中添加 */
// MPC_Ctrl_t    mpc_ctrl;            // MPC运行时数据
// float         MPC_K[4][20];        // 当前腿长下的MPC增益矩阵
// MPC_Mode_e    mpc_mode;            // MPC工作模式
```

把上述定义加入你的 `chassis.h`，在 `Chassis_Param_s` 和 `ChassisInstance` 结构体中分别加入对应字段：

```c
/* ── Chassis_Param_s 结构体中添加 ── */
typedef struct {
    // ... 原有字段 ...
    float LQR_K_Coefficients[40][6];   // 原有LQR拟合系数
    float MPC_K_Coefficients[80][6];   // 新增: MPC增益拟合系数
    float body_mass;
    float track_width;
    float leg_min_length;
    float leg_max_length;
    // ... 其他原有字段 ...
} Chassis_Param_s;

/* ── ChassisInstance 结构体中添加 ── */
typedef struct {
    // ... 原有字段 ...
    float LQR_K[4][10];               // 原有
    float MPC_K[4][MPC_AUG_DIM];      // 新增: MPC增益矩阵
    MPC_Ctrl_t mpc_ctrl;              // 新增: MPC运行时数据
    MPC_Mode_e mpc_mode;              // 新增: MPC模式选择
    // ... 其他原有字段 ...
} ChassisInstance;
```

## 三、`chassis.c` 完整修改

```c
/* ═══════════════════════════════════════════════════════════
 *           增量式MPC相关函数实现
 * ═══════════════════════════════════════════════════════════ */

/**
 * @brief  计算MPC增益矩阵 K_MPC[4][20]
 *
 * 与LQR_K_Calc类似，根据左右腿长通过二维二次多项式插值。
 * K_MPC_ij(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2
 *
 * @param[out] K_MPC  计算得到的MPC增益矩阵 4×20
 * @param[in]  coef   拟合系数 [80][6]
 * @param[in]  l_l    左腿长度
 * @param[in]  l_r    右腿长度
 */
static void MPC_K_Calc(float K_MPC[4][MPC_AUG_DIM],
                        const float coef[80][6],
                        float l_l, float l_r)
{
    for (int n = 0; n < 80; n++) {
        int row = n / MPC_AUG_DIM;  // 0~3 (控制通道)
        int col = n % MPC_AUG_DIM;  // 0~19 (增广状态分量)
        K_MPC[row][col] = coef[n][0]
                        + coef[n][1] * l_l
                        + coef[n][2] * l_r
                        + coef[n][3] * l_l * l_l
                        + coef[n][4] * l_l * l_r
                        + coef[n][5] * l_r * l_r;
    }
}

/**
 * @brief  重置MPC控制器状态
 *
 * 在模式切换、重启、离地等场景调用，防止历史状态导致的跳变。
 */
static void MPC_Reset(MPC_Ctrl_t* mpc)
{
    memset(mpc->x_prev,  0, sizeof(mpc->x_prev));
    memset(mpc->u_prev,  0, sizeof(mpc->u_prev));
    memset(mpc->xi,      0, sizeof(mpc->xi));
    memset(mpc->delta_u, 0, sizeof(mpc->delta_u));
    memset(mpc->u_mpc,   0, sizeof(mpc->u_mpc));
    mpc->initialized = 0;
}

/**
 * @brief  增量式MPC核心计算
 *
 * 增广状态: ξ(k) = [Δx(k); x(k)]  (20×1)
 *   其中 Δx(k) = x(k) - x(k-1)
 *
 * 控制律:
 *   Δu(k) = -K_MPC * ξ(k)      (解析解，无需在线QP)
 *   u(k)  = u(k-1) + Δu(k)     (增量式更新)
 *
 * @param[in]  mpc         MPC运行时数据
 * @param[in]  K_MPC       MPC增益矩阵 [4][20]
 * @param[in]  state_err   当前状态误差向量 [10]  (= x - x_ref)
 * @param[out] u_out       输出控制量 [4]
 */
static void MPC_Incremental_Compute(MPC_Ctrl_t* mpc,
                                     const float K_MPC[4][MPC_AUG_DIM],
                                     const float state_err[MPC_STATE_DIM],
                                     float u_out[MPC_CTRL_DIM])
{
    /* ── 首次运行初始化 ── */
    if (!mpc->initialized) {
        memcpy(mpc->x_prev, state_err, sizeof(float) * MPC_STATE_DIM);
        memset(mpc->u_prev, 0, sizeof(mpc->u_prev));
        mpc->initialized = 1;
    }

    /* ── Step 1: 构建增广状态 ξ(k) = [Δx(k); x(k)] ── */
    for (int i = 0; i < MPC_STATE_DIM; i++) {
        mpc->xi[i]                  = state_err[i] - mpc->x_prev[i];  // Δx(k)
        mpc->xi[i + MPC_STATE_DIM]  = state_err[i];                    // x(k)
    }

    /* ── Step 2: 计算控制增量 Δu(k) = -K_MPC * ξ(k) ── */
    for (int i = 0; i < MPC_CTRL_DIM; i++) {
        mpc->delta_u[i] = 0.0f;
        for (int j = 0; j < MPC_AUG_DIM; j++) {
            mpc->delta_u[i] -= K_MPC[i][j] * mpc->xi[j];
        }
    }

    /* ── Step 3: 增量更新 u(k) = u(k-1) + Δu(k) ── */
    for (int i = 0; i < MPC_CTRL_DIM; i++) {
        mpc->u_mpc[i] = mpc->u_prev[i] + mpc->delta_u[i];
    }

    /* ── Step 4: 输出并保存历史 ── */
    memcpy(u_out, mpc->u_mpc, sizeof(float) * MPC_CTRL_DIM);
    memcpy(mpc->x_prev, state_err, sizeof(float) * MPC_STATE_DIM);
    memcpy(mpc->u_prev, mpc->u_mpc, sizeof(float) * MPC_CTRL_DIM);
}


/* ═══════════════════════════════════════════════════════════
 *     修改后的 LocomotionController (LQR串联MPC)
 * ═══════════════════════════════════════════════════════════ */

static void LocomotionController(void) {
    State_Var_t* sv = &chassis->state_var;
    chassis->update_flag.is_controlled = chassis_ctrl_cmd->vx != 0;

    float l_l = leg[1]->virtual_model.length;
    float l_r = leg[0]->virtual_model.length;

    /* ── 1. 始终计算LQR增益 (作为基线/备用) ── */
    LQR_K_Calc(chassis->LQR_K, chassis->param.LQR_K_Coefficients, l_l, l_r);

    /* ── 2. 构建状态误差向量 (LQR和MPC共用) ── */
    float state_err[10];
    state_err[0] = sv->x_b_h - 0.0f;
    state_err[1] = sv->v_b_h - chassis_ctrl_cmd->vx;
    state_err[2] = sv->phi - chassis_ctrl_cmd->target_yaw;
    VAL_LIMIT(state_err[2], -0.52f, 0.52f);   // ±30°
    state_err[3] = sv->dphi - 0;
    VAL_LIMIT(state_err[3], -2.0f, 2.0f);
    state_err[4] = sv->theta_l - chassis_ctrl_cmd->theta_ff;
    state_err[5] = sv->dtheta_l;
    state_err[6] = sv->theta_r - chassis_ctrl_cmd->theta_ff;
    state_err[7] = sv->dtheta_r;
    state_err[8] = sv->theta_b;
    state_err[9] = sv->dtheta_b;

    /* ── 3. 根据MPC模式选择控制策略 ── */
    float u[4];

    switch (chassis->mpc_mode) {

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     *  模式0: 纯LQR (原有逻辑，不变)
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    case MPC_MODE_OFF:
    default:
    {
        for (int i = 0; i < 4; i++) {
            u[i] = 0.0f;
            for (int j = 0; j < 10; j++) {
                u[i] -= chassis->LQR_K[i][j] * state_err[j];
            }
        }
        /* 同步MPC历史状态，保证切换时无跳变 */
        memcpy(chassis->mpc_ctrl.x_prev, state_err, sizeof(float) * 10);
        memcpy(chassis->mpc_ctrl.u_prev, u, sizeof(float) * 4);
        break;
    }

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     *  模式1: 纯增量式MPC (完全替代LQR)
     *
     *  MPC的R矩阵设计: 轮向权重>>关节权重
     *  → 优化器自然把力矩分配给关节
     *  → 轮向电机期望力矩显著降低
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    case MPC_MODE_PURE:
    {
        /* 计算当前腿长下的MPC增益 */
        MPC_K_Calc(chassis->MPC_K, chassis->param.MPC_K_Coefficients, l_l, l_r);

        /* 增量式MPC计算 */
        MPC_Incremental_Compute(&chassis->mpc_ctrl,
                                (const float (*)[MPC_AUG_DIM])chassis->MPC_K,
                                state_err, u);
        break;
    }

    /* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
     *  模式2: LQR + MPC 混合 (★推荐★)
     *
     *  思路:
     *    1) LQR计算基线控制 u_lqr
     *    2) MPC计算增量优化 u_mpc
     *    3) 关节力矩: 取MPC结果 (MPC更积极使用关节)
     *       轮向力矩: 取两者中绝对值较小的 (保护轮电机)
     *
     *  这样既保留LQR的稳定性保证，
     *  又通过MPC降低轮向力矩需求
     * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
    case MPC_MODE_HYBRID:
    {
        /* (a) LQR基线 */
        float u_lqr[4];
        for (int i = 0; i < 4; i++) {
            u_lqr[i] = 0.0f;
            for (int j = 0; j < 10; j++) {
                u_lqr[i] -= chassis->LQR_K[i][j] * state_err[j];
            }
        }

        /* (b) MPC增量计算 */
        MPC_K_Calc(chassis->MPC_K, chassis->param.MPC_K_Coefficients, l_l, l_r);

        float u_mpc[4];
        MPC_Incremental_Compute(&chassis->mpc_ctrl,
                                (const float (*)[MPC_AUG_DIM])chassis->MPC_K,
                                state_err, u_mpc);

        /* (c) 混合策略 */
        // 关节力矩 (u[0]=右髋, u[1]=左髋): 使用MPC结果
        //   → MPC的R权重对关节宽松，允许关节多出力补偿
        u[0] = u_mpc[0];
        u[1] = u_mpc[1];

        // 轮向力矩 (u[2]=右轮, u[3]=左轮): 取绝对值较小的
        //   → 保护轮电机，防止力矩饱和导致姿态发散
        for (int i = 2; i < 4; i++) {
            if (fabsf(u_mpc[i]) < fabsf(u_lqr[i])) {
                u[i] = u_mpc[i];
            } else {
                u[i] = u_lqr[i];
            }
        }

        /* 可选: 加权混合 (比硬切换更平滑)
         * float alpha = 0.7f;  // MPC权重, 0=纯LQR, 1=纯MPC
         * u[0] = alpha * u_mpc[0] + (1-alpha) * u_lqr[0];  // 右髋
         * u[1] = alpha * u_mpc[1] + (1-alpha) * u_lqr[1];  // 左髋
         * u[2] = alpha * u_mpc[2] + (1-alpha) * u_lqr[2];  // 右轮
         * u[3] = alpha * u_mpc[3] + (1-alpha) * u_lqr[3];  // 左轮
         */

        break;
    }
    } /* end switch */

    /* ── 4. 输出到执行器 (不变) ── */
    leg[0]->virtual_model.Tp = u[0];   // 右髋扭矩
    leg[1]->virtual_model.Tp = u[1];   // 左髋扭矩
    leg[0]->real_model.T     = u[2];   // 右轮扭矩
    leg[1]->real_model.T     = u[3];   // 左轮扭矩
}
```

## 四、初始化和状态管理的修改

在 `ChassisInit` 和 `ChassisCtrlUpdate` 中添加 MPC 相关初始化和重置逻辑：

```c
/* ── ChassisInit() 中添加 ── */
ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config) {
    ChassisInstance* chassis_instance = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));

    // ... 原有初始化代码 ...

    /* ========== MPC初始化 ========== */
    MPC_Reset(&chassis_instance->mpc_ctrl);
    chassis_instance->mpc_mode = MPC_MODE_HYBRID;  // 默认使用混合模式
    // 如果不想用MPC，设为 MPC_MODE_OFF 即回到纯LQR
    /* ================================ */

    // ... 原有初始化代码的剩余部分 ...

    return chassis_instance;
}


/* ── ChassisCtrlUpdate() 中的重启处理 ── */
static void ChassisCtrlUpdate(void) {
    float dt_raw = DWT_GetDeltaT(&chassis->DWT_CNT);

    if (dt_raw > 0.05f) {
        chassis->dt = 0.001f;
        chassis->update_flag.is_restart = 1;
    } else {
        chassis->dt = dt_raw;
        chassis->update_flag.is_restart = 0;
    }

    LegModelUpdate(leg[0], chassis->imu);
    LegModelUpdate(leg[1], chassis->imu);

    StateVarUpdate();

    /* ★ 重启时重置MPC状态，防止历史残留导致跳变 ★ */
    if (chassis->update_flag.is_restart) {
        MPC_Reset(&chassis->mpc_ctrl);
    }

    LocomotionController();
    LegController();

    // ... 跳跃状态机等后续代码不变 ...
}


/* ── ChassisRecovery() 中也需要重置MPC ── */
static void ChassisRecovery(void) {
    chassis_ctrl_cmd->target_yaw = chassis->imu->YawTotalAngle * DEGREE_2_RAD;
    chassis->update_flag.is_controlled = 0;

    /* 恢复模式下清零MPC，防止切回正常模式时跳变 */
    MPC_Reset(&chassis->mpc_ctrl);

    // ... 原有恢复逻辑不变 ...
}
```

## 五、离地保护

在离地检测处也要重置 MPC（防止悬空时历史状态异常）：

```c
/* ── LimitChassisOutput() 中 ── */
static void LimitChassisOutput(void) {
    for (int i = 0; i < 2; i++) {
        VAL_LIMIT(leg[i]->real_model.Tp_1, -33.0f, 33.0f);
        VAL_LIMIT(leg[i]->real_model.Tp_2, -33.0f, 33.0f);
        VAL_LIMIT(leg[i]->real_model.T, -2.45f, 2.45f);

        DMMotorSetRef(leg[i]->joint_motor[0], leg[i]->real_model.Tp_1);
        DMMotorSetRef(leg[i]->joint_motor[1], leg[i]->real_model.Tp_2);

        if (leg[i]->update_flag.is_off_ground) {
            DJIMotorSetRef(leg[i]->wheel_motor, 0);
            /* ★ 任一腿离地时重置MPC ★ */
            MPC_Reset(&chassis->mpc_ctrl);
        } else {
            DJIMotorSetRef(leg[i]->wheel_motor,
                leg[i]->real_model.T * (3591.0f / 187.0f) /
                chassis->leg[i]->param.wheel_reduction_ratio / 0.3f * (16384.0f / 20.0f));
        }
    }
}
```

## 六、MATLAB端：生成MPC拟合系数

在你的 `compute_lqr_mpc.m` 中添加腿长拟合（对应 LQR 的 Step 7），输出格式与 `robot_config.h` 一致：

```matlab
%% ======================== MPC腿长拟合 ========================
% (放在MATLAB脚本的Step 9之前)

fprintf('正在计算不同腿长下的MPC K_mpc矩阵...\n');

K_mpc_sample_2d = zeros(sample_size_2d, 82);  % [l_l, l_r, K_mpc的80个元素]

idx = 0;
for i = 1:num_legs
    for j = 1:num_legs
        idx = idx + 1;
        
        % 左腿参数
        l_l_fit = Leg_data(i, 1);
        l_l_d_fit = Leg_data(i, 2);
        I_l_fit = Leg_data(i, 4);
        
        % 右腿参数
        l_r_fit = Leg_data(j, 1);
        l_r_d_fit = Leg_data(j, 2);
        I_r_fit = Leg_data(j, 4);
        
        % 参数向量
        param_fit = [m_b_val, m_l_val, m_r_val, m_wl_val, m_wr_val, ...
            I_b_val, I_l_fit, I_r_fit, I_wl_val, I_wr_val, I_yaw_val, ...
            l_l_fit, l_r_fit, l_l_d_fit, l_r_d_fit, l_b_val, ...
            R_val, R_w_val, g_val, theta_l0, theta_r0, theta_b0];
        
        A_fit = A_func(param_fit);
        B_fit = B_func(param_fit);
        
        try
            % === 离散化 ===
            sys_c_fit = ss(A_fit, B_fit, eye(n), zeros(n, m));
            sys_d_fit = c2d(sys_c_fit, Ts, 'zoh');
            A_d_fit = sys_d_fit.A;
            B_d_fit = sys_d_fit.B;
            
            % === 增广系统 ===
            A_tilde_fit = [A_d_fit, zeros(n); A_d_fit, eye(n)];
            B_tilde_fit = [B_d_fit; B_d_fit];
            C_tilde_fit = [zeros(n), eye(n)];
            
            % === 预测矩阵 ===
            A_pow = cell(Np+1, 1);
            A_pow{1} = eye(n_aug);
            for kk = 2:Np+1
                A_pow{kk} = A_pow{kk-1} * A_tilde_fit;
            end
            
            Psi_fit = zeros(Np*n, n_aug);
            for kk = 1:Np
                Psi_fit((kk-1)*n+1:kk*n, :) = C_tilde_fit * A_pow{kk+1};
            end
            
            Theta_fit = zeros(Np*n, Np*m);
            for ki = 1:Np
                for kj = 1:ki
                    Theta_fit((ki-1)*n+1:ki*n, (kj-1)*m+1:kj*m) = ...
                        C_tilde_fit * A_pow{ki-kj+1} * B_tilde_fit;
                end
            end
            
            % === 求解析MPC增益 ===
            H_fit = Theta_fit' * Q_bar * Theta_fit + R_bar;
            F_fit = Theta_fit' * Q_bar * Psi_fit;
            K_mpc_fit = (H_fit \ F_fit);
            K_mpc_fit = K_mpc_fit(1:m, :);  % 只取第一步 4×20
            
            % 存储
            K_mpc_sample_2d(idx, 1) = l_l_fit;
            K_mpc_sample_2d(idx, 2) = l_r_fit;
            K_mpc_sample_2d(idx, 3:82) = reshape(K_mpc_fit.', 1, []);
        catch
            warning('MPC计算失败: l_l=%.2f, l_r=%.2f', l_l_fit, l_r_fit);
        end
    end
end

% === 二维多项式拟合 ===
MPC_Fit_Coefficients = zeros(80, 6);
l_l_samples = K_mpc_sample_2d(:, 1);
l_r_samples = K_mpc_sample_2d(:, 2);

for nn = 1:80
    K_values = K_mpc_sample_2d(:, nn+2);
    try
        sf = fit([l_l_samples, l_r_samples], K_values, 'poly22');
        MPC_Fit_Coefficients(nn, :) = coeffvalues(sf);
    catch
        warning('MPC拟合失败: 元素 %d', nn);
    end
end

% === 输出C代码格式 ===
fprintf('\n            .MPC_K_Coefficients = {\n');
for nn = 1:80
    row = ceil(nn/20) - 1;
    col = mod(nn-1, 20);
    fprintf('                {');
    for c = 1:6
        if c < 6
            fprintf('%12.6ff, ', MPC_Fit_Coefficients(nn,c));
        else
            fprintf('%12.6ff', MPC_Fit_Coefficients(nn,c));
        end
    end
    if nn < 80
        fprintf('},  // K_MPC[%d][%d]\n', row, col);
    else
        fprintf('}   // K_MPC[%d][%d]\n', row, col);
    end
end
fprintf('            },\n');
```

## 七、`robot_config.h` 中的配置

```c
/* 在 .param 初始化中添加 MPC_K_Coefficients */
.param = {
    .body_mass = 16.7f,
    .track_width = 0.5005f,
    .leg_min_length = 0.112f,
    .leg_max_length = 0.38f,
    
    .LQR_K_Coefficients = {
        // ... 原有40行LQR拟合系数 (从MATLAB粘贴) ...
    },
    
    .MPC_K_Coefficients = {
        // ★ 从MATLAB compute_lqr_mpc.m 输出粘贴到这里 ★
        // 共80行，每行6个系数
        // K_MPC[i]j [<sup>1</sup>](l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2
        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},  // K_MPC[0][0] — 先占位，MATLAB生成后替换
        // ... 共80行 ...
    },
},
```

## 八、调试建议

```
调试步骤：
┌─────────────────────────────────────────────────────────────┐
│ 1. 先设 mpc_mode = MPC_MODE_OFF                            │
│    确认原有LQR功能完全正常                                    │
│                                                             │
│ 2. MATLAB中仿真对比 LQR vs MPC                              │
│    确认MPC轮向力矩确实更小，且系统稳定                         │
│                                                             │
│ 3. 切 mpc_mode = MPC_MODE_HYBRID                            │
│    先在机器人悬空状态测试（轮子不着地）                         │
│    观察关节和轮向力矩输出是否合理                              │
│                                                             │
│ 4. 着地测试                                                  │
│    观察俯仰角收敛速度、轮向力矩峰值                           │
│    与纯LQR对比                                               │
│                                                             │
│ 5. 调参重点:                                                 │
│    - MATLAB中 R_mpc 的轮向/关节比值 (50/0.5=100倍)          │
│    - Np预测域 (建议2~5)                                      │
│    - Ts采样周期 (与你的控制频率一致)                           │
│    - HYBRID模式下的混合策略 (min选择 vs 加权混合)             │
└─────────────────────────────────────────────────────────────┘
```

## 九、计算量评估

```
纯LQR:    4×10 = 40次乘加                          ≈ 0.8μs @ 168MHz
增量MPC:  4×20 = 80次乘加 + 20次减法(Δx) + 存储    ≈ 2.0μs @ 168MHz
MPC_K_Calc: 80×6 = 480次乘加                        ≈ 5.0μs @ 168MHz
────────────────────────────────────────────────────
总计 MPC 额外开销: ≈ 7μs (相对于1ms控制周期可忽略)

Flash占用: MPC_K_Coefficients[80][6] = 1920 bytes ≈ 1.9KB
RAM占用:   MPC_Ctrl_t ≈ 160 bytes
```

核心结论：增量式MPC的解析解形式使得**MCU端计算量与LQR同量级**，这是它能部署到单片机上的关键优势。R权重的不对称设计（轮向>>关节）是降低轮向力矩的核心机制。
