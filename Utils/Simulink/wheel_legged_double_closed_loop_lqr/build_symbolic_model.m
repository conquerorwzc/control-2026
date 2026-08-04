% build_symbolic_model.m
% 双闭环等效虚拟腿模型: 建立已化简动力学、运动学约束和 M/B/g 函数。
%
% 输入: double_closed_loop_parameters.m 中定义的参数合同。
% 输出: double_closed_loop_symbolic_model.mat。
%
% 模型边界:
%   平地、纯滚动、两轮不离地、无 Roll、固定腿长工作点。
%   左右地面对轮法向力相等是闭合假设，不是由“两轮接地”自动推出的事实。

clear;
clc;

fprintf('========================================\n');
fprintf('双闭环等效腿符号模型构建\n');
fprintf('========================================\n\n');

%% ========================================
%  Step 1: 状态、输入与符号变量
%  ========================================

fprintf('Step 1: 定义状态、输入和物理参数...\n');

syms s dX_b_h ddX_b_h real
syms phi dphi ddphi real
syms theta_l dtheta_l ddtheta_l real
syms theta_r dtheta_r ddtheta_r real
syms theta_b dtheta_b ddtheta_b real
syms T_p_r T_p_l T_w_r T_w_l real

syms g R R_w real
syms m_b m_l m_r m_wl m_wr real
syms I_b I_l I_r I_wl I_wr I_yaw real
syms l_b l_l l_r l_l_d l_r_d theta_l0 theta_r0 theta_b0 real

syms a_b_h a_b_v a_l_h a_l_v a_r_h a_r_v a_wl_h a_wl_v a_wr_h a_wr_v real
syms ddtheta_wl ddtheta_wr real

x = [s; dX_b_h; phi; dphi; theta_l; dtheta_l; theta_r; dtheta_r; theta_b; dtheta_b];
ddq = [ddX_b_h; ddphi; ddtheta_l; ddtheta_r; ddtheta_b];
u = [T_p_r; T_p_l; T_w_r; T_w_l];

fprintf('  x = [s, s_dot, phi, phi_dot, theta_L, theta_L_dot, theta_R, theta_R_dot, theta_b, theta_b_dot]^T\n');
fprintf('  u = [Tp_R, Tp_L, Tw_R, Tw_L]^T\n\n');

%% ========================================
%  Step 2: 已化简的五条动力学方程
%  ========================================

fprintf('Step 2: 建立五条已化简动力学方程...\n');
fprintf('  假设: 左右地面对轮法向力相等，用于闭合左右腿竖直支持力。\n\n');

eq1 = m_b * a_b_h + m_l * a_l_h + m_r * a_r_h + m_wl * a_wl_h + m_wr * a_wr_h ...
    + (T_w_l + T_w_r + I_wl * ddtheta_wl + I_wr * ddtheta_wr) / R;
eq2 = I_b * ddtheta_b - T_p_l - T_p_r - m_b * g * l_b * sin(theta_b + theta_b0);
eq3 = I_r * ddtheta_r - T_w_r + T_p_r - m_r * g * l_r_d * sin(theta_r + theta_r0) ...
    - m_wr * a_wr_h * l_r * cos(theta_r) - (T_w_r + I_wr * ddtheta_wr) * l_r * cos(theta_r) / R ...
    - (m_b * a_b_v + m_l * a_l_v + m_r * a_r_v) * l_r * sin(theta_r) / 2 ...
    - (m_wr - m_b - m_l - m_r - m_wl) * g * l_r * sin(theta_r) / 2;
eq4 = I_l * ddtheta_l - T_w_l + T_p_l - m_l * g * l_l_d * sin(theta_l + theta_l0) ...
    - m_wl * a_wl_h * l_l * cos(theta_l) - (T_w_l + I_wl * ddtheta_wl) * l_l * cos(theta_l) / R ...
    - (m_b * a_b_v + m_l * a_l_v + m_r * a_r_v) * l_l * sin(theta_l) / 2 ...
    - (m_wl - m_b - m_l - m_r - m_wr) * g * l_l * sin(theta_l) / 2;
eq5 = I_yaw * ddphi - R_w / R * (T_w_l + I_wl * ddtheta_wl - T_w_r - I_wr * ddtheta_wr);

fprintf('eq1 (整体水平动量):\n'); disp(eq1);
fprintf('eq2 (机身 Pitch):\n'); disp(eq2);
fprintf('eq3 (右虚拟腿):\n'); disp(eq3);
fprintf('eq4 (左虚拟腿):\n'); disp(eq4);
fprintf('eq5 (整车 Yaw):\n'); disp(eq5);

%% ========================================
%  Step 3: 髋点 O 运动学约束
%  ========================================

fprintf('Step 3: 代入髋点 O 运动学约束...\n');
fprintf('  s = x_O - x_O0; 每侧 x_O = R*theta_w + l*sin(theta_world)。\n');
fprintf('  该式是初始世界前向与车体前向对齐的纵向平面模型；大 Yaw 需另加世界系投影。\n');
fprintf('  固定工作点内忽略 l_dot、l_ddot 的动力学影响。\n\n');

leg_acceleration = (l_r * cos(theta_r) * ddtheta_r + l_l * cos(theta_l) * ddtheta_l) / 2 ...
    - (l_r * sin(theta_r) * dtheta_r^2 + l_l * sin(theta_l) * dtheta_l^2) / 2;
ddtheta_wr_sub = (ddX_b_h + R_w * ddphi - leg_acceleration) / R;
ddtheta_wl_sub = (ddX_b_h - R_w * ddphi - leg_acceleration) / R;

fprintf('ddtheta_wr = '); disp(ddtheta_wr_sub);
fprintf('ddtheta_wl = '); disp(ddtheta_wl_sub);

substitution_lhs = [ddtheta_wr; ddtheta_wl; a_wr_h; a_wl_h; a_wr_v; a_wl_v; ...
    a_r_h; a_l_h; a_r_v; a_l_v; a_b_h; a_b_v];
substitution_rhs = [ddtheta_wr_sub; ddtheta_wl_sub; R * ddtheta_wr_sub; R * ddtheta_wl_sub; ...
    sym(0); sym(0); ...
    R * ddtheta_wr_sub + l_r * cos(theta_r) * ddtheta_r - l_r * sin(theta_r) * dtheta_r^2; ...
    R * ddtheta_wl_sub + l_l * cos(theta_l) * ddtheta_l - l_l * sin(theta_l) * dtheta_l^2; ...
    -l_r * sin(theta_r) * ddtheta_r - l_r * cos(theta_r) * dtheta_r^2; ...
    -l_l * sin(theta_l) * ddtheta_l - l_l * cos(theta_l) * dtheta_l^2; ...
    ddX_b_h; ...
    -(l_r * sin(theta_r) * ddtheta_r + l_l * sin(theta_l) * ddtheta_l) / 2 ...
        - (l_r * cos(theta_r) * dtheta_r^2 + l_l * cos(theta_l) * dtheta_l^2) / 2];

equation = simplify(subs([eq1; eq2; eq3; eq4; eq5], substitution_lhs, substitution_rhs));

%% ========================================
%  Step 4: 提取 M(q)*ddq = B(q)*u + g(q,dq)
%  ========================================

fprintf('Step 4: 提取 M、B 和 g...\n');

M = jacobian(equation, ddq);
B_raw = jacobian(equation, u);
B_control = -B_raw;
g_vector = simplify(-(equation - M * ddq - B_raw * u));
decomposition_residual = simplify(equation - (M * ddq - B_control * u - g_vector));

if isequal(decomposition_residual, sym(zeros(5, 1)))
    fprintf('  ✓ M*ddq - B*u - g 与五条方程严格一致。\n\n');
else
    error('M/B/g decomposition residual is not zero.');
end

fprintf('M(q):\n'); disp(M);
fprintf('B(q):\n'); disp(B_control);
fprintf('g(q,dq):\n'); disp(g_vector);

%% ========================================
%  Step 5: 导出数值函数
%  ========================================

fprintf('Step 5: 创建非线性模型函数...\n');

parameter_symbols = [m_b, m_l, m_r, m_wl, m_wr, I_b, I_l, I_r, I_wl, I_wr, I_yaw, ...
    l_l, l_r, l_l_d, l_r_d, l_b, R, R_w, g, theta_l0, theta_r0, theta_b0];
M_func = matlabFunction(M, 'Vars', {x, parameter_symbols});
B_control_func = matlabFunction(B_control, 'Vars', {x, parameter_symbols});
g_func = matlabFunction(g_vector, 'Vars', {x, parameter_symbols});

save('double_closed_loop_symbolic_model.mat', ...
    'eq1', 'eq2', 'eq3', 'eq4', 'eq5', 'equation', 'substitution_lhs', 'substitution_rhs', ...
    'x', 'u', 'ddq', 'M', 'B_control', 'g_vector', 'parameter_symbols', ...
    'M_func', 'B_control_func', 'g_func');

fprintf('  输出: double_closed_loop_symbolic_model.mat\n');
fprintf('========================================\n');
fprintf('符号模型构建完成\n');
fprintf('========================================\n');
