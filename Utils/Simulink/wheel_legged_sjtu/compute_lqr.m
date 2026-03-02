% compute_lqr.m
% 基于线性化状态空间模型计算LQR控制器增益矩阵
% 
% 依赖文件: linearized_system.mat (由 linearize_system_v2.m 生成)
%
% ========================================================================
%                          变量定义 (对应推导文档 §4.3)
% ========================================================================
%
% 状态向量 X (10维):
%   X = [X_b^h; V_b^h; phi; dphi; theta_l; dtheta_l; theta_r; dtheta_r; theta_b; dtheta_b]
%
%   序号   符号          物理意义                        单位
%   ─────────────────────────────────────────────────────────────────────
%    1    X_b^h        机体水平位置                      m
%    2    V_b^h        机体水平速度 (= dX_b^h/dt)        m/s
%    3    phi          偏航角                            rad
%    4    dphi         偏航角速度                        rad/s
%    5    theta_l      左腿与Z轴负方向夹角               rad
%    6    dtheta_l     左腿角速度                        rad/s
%    7    theta_r      右腿与Z轴负方向夹角               rad
%    8    dtheta_r     右腿角速度                        rad/s
%    9    theta_b      机体俯仰角                        rad
%   10    dtheta_b     机体俯仰角速度                    rad/s
%
% 控制向量 u (4维):
%   u = [T_{r→b}; T_{l→b}; T_{wr→r}; T_{wl→l}]
%
%   序号   符号          物理意义                        执行器
%   ─────────────────────────────────────────────────────────────────────
%    1    T_{r→b}      右腿对机体的扭矩                  右髋关节电机
%    2    T_{l→b}      左腿对机体的扭矩                  左髋关节电机
%    3    T_{wr→r}     右轮对右腿的扭矩                  右轮电机
%    4    T_{wl→l}     左轮对左腿的扭矩                  左轮电机
%
% 扭矩符号约定: T_{A→B} 表示物体A对物体B施加的扭矩
%
% LQR控制律:
%   u = -K * X
%
% 其中 K 为 4×10 增益矩阵
%
% ========================================================================
% 作者: 基于2026公式推导
% 日期: 2026/01/13
% ========================================================================

clear all; clc;
tic

%% ======================== Step 0: 加载线性化系统 ========================

fprintf('========================================\n');
fprintf('轮腿机器人LQR控制器计算\n');
fprintf('========================================\n\n');

fprintf('Step 0: 加载线性化状态空间模型...\n');

% 检查文件是否存在
if ~exist('linearized_system.mat', 'file')
    error('未找到 linearized_system.mat! 请先运行 linearize_system_v2.m');
end

load('linearized_system.mat', 'A_func', 'B_func', 'A_sym', 'B_state_sym', 'param_list');

fprintf('  ✓ 线性化系统加载成功\n');

% 获取函数句柄的输出维度 (22 个参数)
% param_list = [m_b, m_l, m_r, m_wl, m_wr, I_b, I_l, I_r, I_wl, I_wr, I_yaw, l_l, l_r, l_l_d, l_r_d, l_b, R, R_w, g, theta_l0, theta_r0, theta_b0]
test_params = [1, 1, 1, 0.2, 0.2, 0.1, 0.03, 0.03, 0.0002, 0.0002, 0.4, 0.2, 0.2, 0.084, 0.084, 0.05, 0.055, 0.1, -9.81, 0.1, 0.1, 0.05];
A_test = A_func(test_params);
B_test = B_func(test_params);
fprintf('  状态维度: %d\n', size(A_test, 1));
fprintf('  控制维度: %d\n', size(B_test, 2));

%% ======================== Step 1: 定义物理参数 ========================

fprintf('\nStep 1: 定义机器人物理参数...\n');

% linearize_system_v2.m 使用左右腿/轮参数分开模型，参数列表为:
%   [m_b, m_l, m_r, m_wl, m_wr, I_b, I_l, I_r, I_wl, I_wr, I_yaw, l_l, l_r, l_l_d, l_r_d, l_b, R, R_w, g, theta_l0, theta_r0, theta_b0]
% 其中左右腿/轮参数可以独立设置（支持不对称）

% ==================== 物理常数 ====================
g_val = -9.81;              % 重力加速度 (m/s^2)

% ==================== 几何参数 ====================
R_val = 0.077;              % 轮子半径 (m) — SRM
R_w_val = 0.25025;          % 轮距/2 (m) — SRM

% ==================== 机体参数 ====================
m_b_val = 16.7;             % 机体质量 (kg) — SRM
I_b_val = 0.3160;           % 机体俯仰转动惯量 (kg·m²) — SRM
l_b_val = 0.0;              % 机体质心到俯仰轴距离 (m) — SRM
I_yaw_val = 0.5965;         % 整体yaw轴转动惯量 (kg·m²) — SRM
theta_b0 = 0;               % 质心偏移角度，单位：弧度 — SRM

% ==================== 轮子参数 (分别定义左右) ====================
m_w_val = 0.7;              % 单轮质量 (kg) — SRM
m_wl_val = m_w_val;         % 左轮质量 (kg)
m_wr_val = m_w_val;         % 右轮质量 (kg)
I_wl_val = 0.5*m_w_val*R_val^2;  % 左轮转动惯量 (kg·m²) — 0.5*m_w*R²
I_wr_val = 0.5*m_w_val*R_val^2;  % 右轮转动惯量 (kg·m²) — 0.5*m_w*R²

% ==================== 腿部参数 (分别定义左右, 默认腿长 0.20m) ====================
m_leg_val = 1.3;            % 单腿等效质量 (kg) — SRM
l_l_val = 0.20;             % 左腿长度 (m)
l_r_val = 0.20;             % 右腿长度 (m)
m_l_val = m_leg_val;        % 左腿质量 (kg)
m_r_val = m_leg_val;        % 右腿质量 (kg)
% 腿部惯量: m_leg*((l_leg)^2+0.05^2)/12.0
I_l_val = m_leg_val*((l_l_val)^2+0.05^2)/12.0;
I_r_val = m_leg_val*((l_r_val)^2+0.05^2)/12.0;
l_l_d_val = 0.5 * l_l_val;     % 左腿质心距离 = 0.5 * l_leg — SRM
l_r_d_val = 0.5 * l_r_val;     % 右腿质心距离 = 0.5 * l_leg — SRM
theta_l0 = 0;                   % 左腿偏移角度 — SRM
theta_r0 = 0;                   % 右腿偏移角度 — SRM

fprintf('  ✓ 物理参数设置完成\n');

%% ======================== Step 2: 数值代入 ========================

fprintf('\nStep 2: 代入数值参数...\n');

% 参数值向量 (顺序与 linearize_system_v2.m 中 param_list 一致)
%   param_list = [m_b, m_l, m_r, m_wl, m_wr, I_b, I_l, I_r, I_wl, I_wr, I_yaw, l_l, l_r, l_l_d, l_r_d, l_b, R, R_w, g, theta_l0, theta_r0, theta_b0]
param_vals = [m_b_val, m_l_val, m_r_val, m_wl_val, m_wr_val, I_b_val, I_l_val, I_r_val, I_wl_val, I_wr_val, I_yaw_val, ...
              l_l_val, l_r_val, l_l_d_val, l_r_d_val, l_b_val, R_val, R_w_val, g_val, theta_l0, theta_r0, theta_b0];

% 使用函数句柄计算数值矩阵
A_num = A_func(param_vals);
B_num = B_func(param_vals);

fprintf('  ✓ 数值代入完成\n');

% 显示矩阵
fprintf('\n  数值A矩阵 (10×10):\n');
disp(A_num);
fprintf('  数值B矩阵 (10×4):\n');
disp(B_num);

%% ======================== Step 3: 检查可控性 ========================

fprintf('Step 3: 检查系统可控性...\n');

Co = ctrb(A_num, B_num);
rank_Co = rank(Co);
fprintf('  可控性矩阵秩: %d (系统维度: 10)\n', rank_Co);

if rank_Co < 10
    fprintf('\n  ⚠ 系统不完全可控 (秩=%d < 10)\n', rank_Co);
    fprintf('  物理原因: X_b^h(水平位置) 和 phi(yaw角) 是积分器状态\n');
    fprintf('           机器人可以在任意位置/朝向平衡，这两个状态不影响动力学\n');
    fprintf('  解决方案: 这是正常的! LQR仍然可以计算可控子空间的增益\n\n');
else
    fprintf('  ✓ 系统完全可控\n\n');
end

%% ======================== Step 4: 设置LQR权重 ========================

fprintf('Step 4: 设置LQR权重矩阵...\n');

% % Q矩阵: 状态权重
% % 状态: [X_b^h, V_b^h, phi, dphi, theta_l, dtheta_l, theta_r, dtheta_r, theta_b, dtheta_b]
% %            位置    速度      偏航   偏航速    左腿角    左腿速     右腿角    右腿速     俯仰角   俯仰速
lqr_Q = diag([100,    1,      4000,    1,      1000,     10,       1000,     10,       40000,    1]);
lqr_R = diag([1,      1,        10,        10]);
% 
% lqr_Q = diag([300,    300,      600,    1,      10,     10,       10,     10,       6000,    10]);
% R矩阵: 控制输入权重
% 控制: [T_{r→b}, T_{l→b}, T_{wr→r}, T_{wl→l}]
%        右髋扭矩   左髋扭矩   右轮扭矩   左轮扭矩
% lqr_R = diag([0.25,      0.25,        1,        1]);

fprintf('  Q矩阵 (状态权重):\n');
fprintf('         X_b^h  V_b^h  phi   dphi  θ_l   dθ_l  θ_r   dθ_r  θ_b   dθ_b\n');
disp(lqr_Q);

fprintf('  R矩阵 (控制权重):\n');
fprintf('         T_{r→b}  T_{l→b}  T_{wr→r}  T_{wl→l}\n');
disp(lqr_R);

%% ======================== Step 5: 计算LQR增益 ========================

fprintf('Step 5: 计算LQR增益矩阵K...\n');

try
    [K, S, e] = lqr(A_num, B_num, lqr_Q, lqr_R);
    
    fprintf('\n  ✓ LQR增益矩阵K (4×10):\n');
    disp(K);
    
    fprintf('  闭环特征值:\n');
    disp(e);
    
    % 检查稳定性
    stable_eigs = real(e) < 1e-6;
    if all(stable_eigs)
        fprintf('  ✓ 闭环系统稳定!\n\n');
    else
        warning('闭环系统不稳定!');
    end
catch ME
    fprintf('  ✗ LQR计算失败: %s\n', ME.message);
    K = [];
end

%% ======================== Step 6: 格式化输出 ========================

fprintf('========================================\n');
fprintf('格式化输出 (可直接复制到C代码)\n');
fprintf('========================================\n\n');

if ~isempty(K)
    % K矩阵行列含义
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// LQR增益矩阵 K[4][10]\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// 控制律: u = -K * X\n');
    fprintf('//\n');
    fprintf('// 状态向量 X (列向量 10×1):\n');
    fprintf('//   X[0] = X_b^h     机体水平位置 (m)\n');
    fprintf('//   X[1] = V_b^h     机体水平速度 (m/s)\n');
    fprintf('//   X[2] = phi       偏航角 (rad)\n');
    fprintf('//   X[3] = dphi      偏航角速度 (rad/s)\n');
    fprintf('//   X[4] = theta_l   左腿角 (rad)\n');
    fprintf('//   X[5] = dtheta_l  左腿角速度 (rad/s)\n');
    fprintf('//   X[6] = theta_r   右腿角 (rad)\n');
    fprintf('//   X[7] = dtheta_r  右腿角速度 (rad/s)\n');
    fprintf('//   X[8] = theta_b   机体俯仰角 (rad)\n');
    fprintf('//   X[9] = dtheta_b  机体俯仰角速度 (rad/s)\n');
    fprintf('//\n');
    fprintf('// 控制向量 u (列向量 4×1):\n');
    fprintf('//   u[0] = T_r_to_b   右髋扭矩 (右腿→机体) (Nm)\n');
    fprintf('//   u[1] = T_l_to_b   左髋扭矩 (左腿→机体) (Nm)\n');
    fprintf('//   u[2] = T_wr_to_r  右轮扭矩 (右轮→右腿) (Nm)\n');
    fprintf('//   u[3] = T_wl_to_l  左轮扭矩 (左轮→左腿) (Nm)\n');
    fprintf('//\n');
    fprintf('// K矩阵含义:\n');
    fprintf('//   K[i][j] 表示控制输入 u[i] 对状态 X[j] 的反馈增益\n');
    fprintf('//   K[0][*]: 右髋扭矩对各状态的增益\n');
    fprintf('//   K[1][*]: 左髋扭矩对各状态的增益\n');
    fprintf('//   K[2][*]: 右轮扭矩对各状态的增益\n');
    fprintf('//   K[3][*]: 左轮扭矩对各状态的增益\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n\n');
    
    fprintf('float K[4][10] = {\n');
    control_names = {'T_r_to_b', 'T_l_to_b', 'T_wr_to_r', 'T_wl_to_l'};
    for i = 1:4
        fprintf('    {%11.6ff, %11.6ff, %11.6ff, %11.6ff, %11.6ff, ', K(i,1), K(i,2), K(i,3), K(i,4), K(i,5));
        fprintf('%11.6ff, %11.6ff, %11.6ff, %11.6ff, %11.6ff}', K(i,6), K(i,7), K(i,8), K(i,9), K(i,10));
        if i < 4
            fprintf(',  // %s\n', control_names{i});
        else
            fprintf('   // %s\n', control_names{i});
        end
    end
    fprintf('};\n\n');
    
    % 单行格式
    fprintf('// 单行格式 (每行对应一个控制输入):\n');
    for i = 1:4
        fprintf('// K[%d] (%s): ', i-1, control_names{i});
        fprintf('%.6g, ', K(i,1:9));
        fprintf('%.6g\n', K(i,10));
    end
    fprintf('\n');
end

%% ======================== Step 7: 腿长拟合功能 ========================

fprintf('========================================\n');
fprintf('Step 7: 腿长拟合功能\n');
fprintf('========================================\n\n');

% 腿长参数查找表 — SRM robot
% 格式: [腿长(m), 质心到轮轴距离(m), (unused), 转动惯量(kg·m²)]
% l_leg_d = 0.5 * l_leg
% I_leg   = m_leg * ((l_leg)^2 + 0.05^2) / 12.0
leg_lengths = (0.112:0.01:0.38)';
Leg_data = zeros(length(leg_lengths), 4);
for idx = 1:length(leg_lengths)
    ll = leg_lengths(idx);
    Leg_data(idx, 1) = ll;                                      % 腿长 (m)
    Leg_data(idx, 2) = 0.5 * ll;                                % 质心距离 l_leg_d
    Leg_data(idx, 3) = 0.5 * ll;                                % (same as col 2 for this model)
    Leg_data(idx, 4) = m_leg_val * ((ll)^2 + 0.05^2) / 12.0;   % 转动惯量 I_leg
end

enable_fitting = true;  % 设为 false 跳过腿长拟合

if enable_fitting
    fprintf('正在计算不同腿长下的K矩阵...\n');
    fprintf('  注意: 使用左右腿参数分开模型，支持左右腿长不同\n\n');
    
    % ========== 计算采样点 (二维网格) ==========
    num_legs = size(Leg_data, 1);
    sample_size_2d = num_legs^2;
    
    % K矩阵 4×10 = 40 个元素
    % 二维拟合: [l_l, l_r, K矩阵的40个元素]
    K_sample_2d = zeros(sample_size_2d, 44);  % [l_l, l_r, K矩阵的40个元素]
    
    tic_fit = tic;
    
    idx = 0;
    for i = 1:num_legs
        for j = 1:num_legs
            idx = idx + 1;
            
            % 左腿参数
            l_l_fit = Leg_data(i, 1);
            l_l_d_fit = Leg_data(i, 2);   % 0.5 * l_leg
            I_l_fit = Leg_data(i, 4);
            
            % 右腿参数
            l_r_fit = Leg_data(j, 1);
            l_r_d_fit = Leg_data(j, 2);   % 0.5 * l_leg
            I_r_fit = Leg_data(j, 4);
            
            % 构建参数向量
            % param_list = [m_b, m_l, m_r, m_wl, m_wr, I_b, I_l, I_r, I_wl, I_wr, I_yaw, l_l, l_r, l_l_d, l_r_d, l_b, R, R_w, g, theta_l0, theta_r0, theta_b0]
            param_fit = [m_b_val, m_l_val, m_r_val, m_wl_val, m_wr_val, I_b_val, I_l_fit, I_r_fit, I_wl_val, I_wr_val, I_yaw_val, ...
                         l_l_fit, l_r_fit, l_l_d_fit, l_r_d_fit, l_b_val, R_val, R_w_val, g_val, theta_l0, theta_r0, theta_b0];
            
            % 计算数值矩阵
            A_fit = A_func(param_fit);
            B_fit = B_func(param_fit);
            
            % 计算LQR
            try
                K_fit = lqr(A_fit, B_fit, lqr_Q, lqr_R);
                
                % 存储结果
                K_sample_2d(idx, 1) = l_l_fit;
                K_sample_2d(idx, 2) = l_r_fit;
                K_sample_2d(idx, 3:42) = reshape(K_fit.', 1, []);  % 真正的行优先展开
            catch
                warning('LQR计算失败: l_l=%.2f, l_r=%.2f', l_l_fit, l_r_fit);
            end
            
            % 显示进度
            if mod(idx, 49) == 0
                fprintf('  进度: %d/%d (%.1f秒)\n', idx, sample_size_2d, toc(tic_fit));
            end
        end
    end
    
    fprintf('  ✓ %d 个样本计算完成! 耗时: %.2f秒\n', sample_size_2d, toc(tic_fit));
    
    % ========== 二维多项式拟合 ==========
    fprintf('\n正在进行二维多项式拟合...\n');
    
    % 拟合多项式: K_ij(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2
    K_Fit_Coefficients = zeros(40, 6);
    
    l_l_samples = K_sample_2d(:, 1);
    l_r_samples = K_sample_2d(:, 2);
    
    for n = 1:40
        K_values = K_sample_2d(:, n+2);
        try
            % 二维二次多项式拟合
            K_Surface_Fit = fit([l_l_samples, l_r_samples], K_values, 'poly22');
            coeffs = coeffvalues(K_Surface_Fit);
            K_Fit_Coefficients(n, :) = coeffs;  % [p00, p10, p01, p20, p11, p02]
        catch
            warning('二维拟合失败: 元素 %d', n);
        end
    end
    
    fprintf('  ✓ 拟合完成\n\n');
    
    % ========== 输出拟合系数 (robot_config.h format) ==========
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// 腿长拟合系数 LQR_K_Coefficients[40][6] (左右腿可不同)\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// 拟合多项式 (二维):\n');
    fprintf('//   K_ij(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2\n');
    fprintf('//\n');
    fprintf('// K元素排列 (按行优先):\n');
    fprintf('//   n=0~9:   K[0][0~9] → T_{r→b} 对各状态的增益\n');
    fprintf('//   n=10~19: K[1][0~9] → T_{l→b} 对各状态的增益\n');
    fprintf('//   n=20~29: K[2][0~9] → T_{wr→r} 对各状态的增益\n');
    fprintf('//   n=30~39: K[3][0~9] → T_{wl→l} 对各状态的增益\n');
    fprintf('//\n');
    fprintf('// 系数顺序: [p00, p10, p01, p20, p11, p02]\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n\n');
    
    % 生成与 robot_config.h 完全一致的 C 代码 (Chassis_Param_s.LQR_K_Coefficients[40][6])
    % 缩进与 infantry_wheel_legged_sjtu/robot_config.h 中 .param 内一致，可直接替换 .LQR_K_Coefficients = {{0}},
    paste_lines = {};
    paste_lines{1} = '            .LQR_K_Coefficients = {';
    for n = 1:40
        row = ceil(n/10) - 1;  % 0-indexed
        col = mod(n-1, 10);    % 0-indexed
        line = '                {';
        for c = 1:6
            if c < 6
                line = [line, sprintf('%12.6ff, ', K_Fit_Coefficients(n,c))];
            else
                line = [line, sprintf('%12.6ff', K_Fit_Coefficients(n,c))];
            end
        end
        if n < 40
            line = [line, sprintf('},  // K[%d][%d]', row, col)];
        else
            line = [line, sprintf('}   // K[%d][%d]', row, col)];
        end
        paste_lines{end+1} = line;
    end
    paste_lines{end+1} = '            },';
    paste_block = strjoin(paste_lines, '\n');

    fprintf('// --- 以下格式与 robot_config.h 中 .param 内 .LQR_K_Coefficients 一致，可整体替换 .LQR_K_Coefficients = {{0}}, ---\n\n');
    fprintf('%s\n\n', paste_block);

    % 复制到剪贴板，便于直接粘贴到 robot_config.h
    try
        clipboard('copy', paste_block);
        fprintf('  [已复制到剪贴板] 可直接在 robot_config.h 中选中 ".LQR_K_Coefficients = {{0}}," 并粘贴替换。\n\n');
    catch
        fprintf('  [剪贴板写入失败] 请从上方输出手动复制到 robot_config.h。\n\n');
    end
    
    % ========== 验证: K矩阵符号检查 ==========
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// K矩阵符号检查 (用默认腿长 l_l=l_r=0.20m 的K矩阵)\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('// 预期物理行为:\n');
    fprintf('//   - 机体后仰(theta_b>0) → 前轮加速(T_wr>0, T_wl>0) → K[2][8]<0, K[3][8]<0\n');
    fprintf('//   - 前进速度偏大(V_b_h>0) → 减速 → K[2][1]>0, K[3][1]>0\n');
    fprintf('//   - 左腿后摆(theta_l>0) → 左髋反向 → K[1][4]<0\n');
    fprintf('// ═══════════════════════════════════════════════════════════════════════\n');
    fprintf('K at default leg length:\n');
    disp(K);
    
    % 保存拟合结果
    save('lqr_fitting_results.mat', 'K_sample_2d', 'K_Fit_Coefficients', 'Leg_data');
    fprintf('拟合结果已保存到 lqr_fitting_results.mat (左右腿独立参数版本)\n');
end

%% ======================== Step 8: 保存结果 ========================

fprintf('\n========================================\n');
fprintf('保存结果\n');
fprintf('========================================\n\n');

save('lqr_results.mat', 'A_num', 'B_num', 'K', 'lqr_Q', 'lqr_R', 'e');
fprintf('✓ LQR结果已保存到 lqr_results.mat\n');

elapsed_time = toc;
fprintf('\n计算完成! 总耗时: %.2f秒\n', elapsed_time);
