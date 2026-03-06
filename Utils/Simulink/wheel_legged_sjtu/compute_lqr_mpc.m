% compute_lqr_mpc.m
% ========================================================================
% LQR串联增量式MPC控制器计算 (带腿长拟合功能)
%
% 参考: 上海交通大学交龙战队 / 电子科技大学中山学院柳工开源
% 核心思想: 通过增量式MPC削弱关节力矩以降低轮向电机的期望力矩
%
% 功能:
% 1. 在腿长网格上采样，计算不同腿长下的 LQR 和 MPC 增益
% 2. 对增益矩阵进行二维多项式拟合
% 3. 生成可直接粘贴到 robot_config.h 的 C 代码
%
% ========================================================================

clear all; clc;
tic

%% ======================== Step 0: 加载线性化系统 ========================

fprintf('════════════════════════════════════════════════════\n');
fprintf(' 轮腿机器人 LQR & MPC 参数拟合生成脚本\n');
fprintf('════════════════════════════════════════════════════\n\n');

fprintf('Step 0: 加载线性化状态空间模型...\n');

if ~exist('linearized_system.mat', 'file')
    error('未找到 linearized_system.mat! 请先运行 linearize_system_v2.m');
end

load('linearized_system.mat', 'A_func', 'B_func', 'A_sym', 'B_state_sym', 'param_list');
fprintf(' ✓ 线性化系统加载成功\n');

%% ======================== Step 1: 定义物理常数与默认参数 ========================

fprintf('\nStep 1: 定义物理参数...\n');

% ==================== 物理常数 ====================
g_val = -9.81;

% ==================== 几何参数 ====================
R_val = 0.077;          % 轮子半径 (m)
R_w_val = 0.25025;      % 轮距/2 (m)

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

% ==================== 腿部参数 (基础值) ====================
m_leg_val = 1.3;
m_l_val = m_leg_val;
m_r_val = m_leg_val;
theta_l0 = 0;
theta_r0 = 0;

fprintf(' ✓ 物理参数设置完成\n');

%% ======================== Step 2: 定义控制器参数 ========================

fprintf('\nStep 2: 定义控制器权重参数...\n');

% ===== LQR 权重 =====
% 状态: [X_b^h, V_b^h, phi, dphi, theta_l, dtheta_l, theta_r, dtheta_r, theta_b, dtheta_b]
lqr_Q = diag([400, 300, 600, 1, 10, 10, 10, 10, 5000, 1]);
lqr_R = diag([1, 1, 8, 8]);

% ===== MPC 参数 =====
Ts = 0.002;  % 采样周期 (s)
Np = 3;      % 预测域

% MPC 状态权重 (Q_mpc)
% 强调俯仰角平衡
Q_mpc = diag([200, 200, 400, 1, 10, 1, 10, 1, 8000, 30]);
Q_terminal = 2.0 * Q_mpc; % 终端权重

% MPC 控制权重 (R_mpc)
% 关键: 轮向力矩权重远大于关节力矩权重 -> 削弱轮向力矩
% R_mpc = diag([0.5, 0.5, 50, 50]);
R_mpc = diag([1, 1, 15, 15]);

fprintf(' ✓ 控制器参数设置完成\n');

%% ======================== Step 3: 腿长采样与增益计算 ========================

fprintf('\nStep 3: 开始网格采样计算 (LQR & MPC)...\n');

% 腿长范围 (m)
leg_lengths = (0.112:0.01:0.38)';
num_legs = length(leg_lengths);
sample_size = num_legs^2;

% 存储采样数据
% Col 1: l_l, Col 2: l_r
% Col 3~42: LQR_K (40个元素)
% Col 43~122: MPC_K (80个元素)
Data_Samples = zeros(sample_size, 2 + 40 + 80);

idx = 0;
tic_loop = tic;

for i = 1:num_legs
    for j = 1:num_legs
        idx = idx + 1;

        % 当前腿长
        l_l_cur = leg_lengths(i);
        l_r_cur = leg_lengths(j);

        % 计算腿部惯量等衍生参数
        I_l_cur = m_leg_val * ((l_l_cur)^2 + 0.05^2) / 12.0;
        I_r_cur = m_leg_val * ((l_r_cur)^2 + 0.05^2) / 12.0;
        l_l_d_cur = 0.5 * l_l_cur;
        l_r_d_cur = 0.5 * l_r_cur;

        % 构建参数向量
        param_vals = [m_b_val, m_l_val, m_r_val, m_wl_val, m_wr_val, ...
            I_b_val, I_l_cur, I_r_cur, I_wl_val, I_wr_val, I_yaw_val, ...
            l_l_cur, l_r_cur, l_l_d_cur, l_r_d_cur, l_b_val, ...
            R_val, R_w_val, g_val, theta_l0, theta_r0, theta_b0];

        % 计算连续系统矩阵
        A_num = A_func(param_vals);
        B_num = B_func(param_vals);

        n = size(A_num, 1);
        m = size(B_num, 2);

        % ---------------- LQR 计算 ----------------
        try
            K_lqr = lqr(A_num, B_num, lqr_Q, lqr_R);
        catch
            K_lqr = zeros(m, n);
            warning('LQR failed at l_l=%.3f, l_r=%.3f', l_l_cur, l_r_cur);
        end

        % ---------------- MPC 计算 ----------------
        try
            % 离散化
            sys_c = ss(A_num, B_num, eye(n), zeros(n, m));
            sys_d = c2d(sys_c, Ts, 'zoh');
            A_d = sys_d.A;
            B_d = sys_d.B;

            % 构建增广系统
            A_tilde = [A_d, zeros(n); A_d, eye(n)];
            B_tilde = [B_d; B_d];
            C_tilde = [zeros(n), eye(n)];
            n_aug = 2 * n;

            % 预测矩阵
            Psi = zeros(Np*n, n_aug);
            Theta = zeros(Np*n, Np*m);
            A_tilde_pow = eye(n_aug);

            % 预计算幂次并填充 Psi
            A_pow_store = cell(Np, 1);
            for k = 1:Np
                A_tilde_pow = A_tilde_pow * A_tilde;
                A_pow_store{k} = A_tilde_pow;
                Psi((k-1)*n+1 : k*n, :) = C_tilde * A_tilde_pow;
            end

            % 填充 Theta
            for r = 1:Np
                for c = 1:r
                    if r-c == 0
                        term = C_tilde * B_tilde;
                    else
                        term = C_tilde * A_pow_store{r-c} * B_tilde;
                    end
                    Theta((r-1)*n+1 : r*n, (c-1)*m+1 : c*m) = term;
                end
            end

            % 构建代价矩阵
            Q_bar = kron(eye(Np), Q_mpc);
            Q_bar((Np-1)*n+1:Np*n, (Np-1)*n+1:Np*n) = Q_terminal;
            R_bar = kron(eye(Np), R_mpc);

            % 解析解
            H = Theta' * Q_bar * Theta + R_bar;
            F = Theta' * Q_bar * Psi;

            % 求逆 (注意: 如果H接近奇异可能会出问题，这里假设正定)
            % 实际上 H 是对称正定的，可以用 cholesky 分解加速，这里直接 inv
            K_mpc_full = H \ F;
            K_mpc = K_mpc_full(1:m, :);

        catch ME
            K_mpc = zeros(m, 2*n);
            warning('MPC failed at l_l=%.3f, l_r=%.3f: %s', l_l_cur, l_r_cur, ME.message);
        end

        % ---------------- 存储数据 ----------------
        Data_Samples(idx, 1) = l_l_cur;
        Data_Samples(idx, 2) = l_r_cur;
        Data_Samples(idx, 3:42) = reshape(K_lqr.', 1, []); % 行优先展开
        Data_Samples(idx, 43:122) = reshape(K_mpc.', 1, []); % 行优先展开

        if mod(idx, 100) == 0
            fprintf('  进度: %d/%d (%.1f%%)\n', idx, sample_size, idx/sample_size*100);
        end
    end
end

fprintf(' ✓ 采样完成，耗时: %.2f秒\n', toc(tic_loop));

%% ======================== Step 4: 多项式拟合 ========================

fprintf('\nStep 4: 进行二维多项式拟合 (Poly22)...\n');

% 拟合模型: f(x,y) = p00 + p10*x + p01*y + p20*x^2 + p11*x*y + p02*y^2
% x = l_l, y = l_r

l_l_data = Data_Samples(:, 1);
l_r_data = Data_Samples(:, 2);

% --- LQR 拟合 ---
LQR_Fit_Coeffs = zeros(40, 6);
fprintf('  拟合 LQR 参数 (40个)...\n');
for k = 1:40
    vals = Data_Samples(:, 2 + k);
    try
        ft = fit([l_l_data, l_r_data], vals, 'poly22');
        LQR_Fit_Coeffs(k, :) = coeffvalues(ft);
    catch
        warning('LQR拟合失败: index %d', k);
    end
end

% --- MPC 拟合 ---
MPC_Fit_Coeffs = zeros(80, 6);
fprintf('  拟合 MPC 参数 (80个)...\n');
for k = 1:80
    vals = Data_Samples(:, 42 + k);
    try
        ft = fit([l_l_data, l_r_data], vals, 'poly22');
        MPC_Fit_Coeffs(k, :) = coeffvalues(ft);
    catch
        warning('MPC拟合失败: index %d', k);
    end
end

fprintf(' ✓ 拟合完成\n');

%% ======================== Step 5: 生成代码 ========================

fprintf('\nStep 5: 生成 C 代码...\n');
fprintf('======================================================================\n');
fprintf('请复制以下内容到 robot_config.h 对应位置\n');
fprintf('======================================================================\n\n');

% 生成 LQR 代码块
lqr_lines = {};
lqr_lines{1} = '            .LQR_K_Coefficients = {';
for n = 1:40
    row = ceil(n/10) - 1;
    col = mod(n-1, 10);
    line_str = '                {';
    for c = 1:6
        if c < 6
            line_str = [line_str, sprintf('%12.6ff, ', LQR_Fit_Coeffs(n,c))];
        else
            line_str = [line_str, sprintf('%12.6ff', LQR_Fit_Coeffs(n,c))];
        end
    end
    if n < 40
        line_str = [line_str, sprintf('},  // LQR K[%d][%d]', row, col)];
    else
        line_str = [line_str, sprintf('}   // LQR K[%d][%d]', row, col)];
    end
    lqr_lines{end+1} = line_str;
end
lqr_lines{end+1} = '            },';

% 生成 MPC 代码块
mpc_lines = {};
mpc_lines{1} = '            .MPC_K_Coefficients = {';
for n = 1:80
    row = ceil(n/20) - 1;
    col = mod(n-1, 20);
    line_str = '                {';
    for c = 1:6
        if c < 6
            line_str = [line_str, sprintf('%12.6ff, ', MPC_Fit_Coeffs(n,c))];
        else
            line_str = [line_str, sprintf('%12.6ff', MPC_Fit_Coeffs(n,c))];
        end
    end
    if n < 80
        line_str = [line_str, sprintf('},  // MPC K[%d][%d]', row, col)];
    else
        line_str = [line_str, sprintf('}   // MPC K[%d][%d]', row, col)];
    end
    mpc_lines{end+1} = line_str;
end
mpc_lines{end+1} = '            },';

% 输出到终端
final_str = [strjoin(lqr_lines, '\n'), sprintf('\n'), strjoin(mpc_lines, '\n')];
fprintf('%s\n', final_str);

% 尝试复制到剪贴板
try
    clipboard('copy', final_str);
    fprintf('\n[提示] 代码已复制到剪贴板！\n');
catch
    fprintf('\n[提示] 无法写入剪贴板，请手动复制。\n');
end

fprintf('\n计算结束。\n');
