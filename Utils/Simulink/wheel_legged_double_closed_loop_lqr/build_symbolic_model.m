function build_symbolic_model()
% build_symbolic_model.m
% 双闭环等效虚拟腿模型：由原始 Newton-Euler 方程自动消元生成 M、B、g。
%
% 输入：double_closed_loop_parameters.m 中定义的参数合同。
% 输出：double_closed_loop_symbolic_model.mat。
%
% 模型边界：
%   平地、纯滚动、双轮接地、无 Roll、固定腿长工作点。
%   以左右法向力相等闭合竖直内力；该条件不由“双轮接地”自动推出。

clc;

fprintf('========================================\n');
fprintf('双闭环等效腿符号模型构建\n');
fprintf('========================================\n\n');

%% ========================================
%  Step 1: 状态、输入和物理参数
%  ========================================

fprintf('Step 1: 定义状态、输入和物理参数...\n');

syms s ds dds real
syms phi dphi ddphi real
syms theta_l dtheta_l ddtheta_l real
syms theta_r dtheta_r ddtheta_r real
syms theta_b dtheta_b ddtheta_b real
syms T_p_r T_p_l T_w_r T_w_l real

syms g wheel_radius half_track real
syms m_b m_l m_r m_wl m_wr real
syms I_b I_l I_r I_wl I_wr I_yaw real
syms l_b l_l l_r l_l_d l_r_d theta_l0 theta_r0 theta_b0 real

syms a_b_h a_b_v a_l_h a_l_v a_r_h a_r_v a_wl_h a_wl_v a_wr_h a_wr_v real
syms ddtheta_wl ddtheta_wr real

% 机体-腿、腿-轮和地面接触力均是待消去的内力；Tp、Tw 是外部控制输入。
syms F_l_to_b_h F_l_to_b_v F_r_to_b_h F_r_to_b_v real
syms F_wl_to_l_h F_wl_to_l_v F_wr_to_r_h F_wr_to_r_v real
syms F_g_to_wl_h F_g_to_wl_v F_g_to_wr_h F_g_to_wr_v real

% Tw 是电机施加给轮子的轮端力矩，正值使对应轮向前滚动。
% 电机定子固定在腿上，因此定子对腿的反作用力矩与 Tw 符号相反。
% 该力矩不是轮轴传力 F_w_to_l，二者必须独立保留，避免与 SJTU/WBR 的内部力矩混淆。
T_motor_stator_to_left_leg = -T_w_l;
T_motor_stator_to_right_leg = -T_w_r;

x = [s; ds; phi; dphi; theta_l; dtheta_l; theta_r; dtheta_r; theta_b; dtheta_b];
ddq = [dds; ddphi; ddtheta_l; ddtheta_r; ddtheta_b];
u = [T_p_r; T_p_l; T_w_r; T_w_l];

fprintf('  x = [s, s_dot, phi, phi_dot, theta_L, theta_L_dot, theta_R, theta_R_dot, theta_b, theta_b_dot]^T\n');
fprintf('  u = [Tp_R, Tp_L, Tw_R, Tw_L]^T\n');
fprintf('  Tw_i > 0 定义为对第 i 个轮施加向前滚动的轮端力矩。\n\n');

%% ========================================
%  Step 2: 原始 Newton-Euler 方程
%  ========================================

fprintf('Step 2: 建立原始 Newton-Euler 方程和等法向力闭合...\n');

% 每条腿以自身质心为转动中心：l_d 是轮轴到质心，l-l_d 是质心到髋点。
left_com_to_hip = l_l - l_l_d;
right_com_to_hip = l_r - l_r_d;

raw_body_h = F_l_to_b_h + F_r_to_b_h - m_b * a_b_h;
raw_body_v = F_l_to_b_v + F_r_to_b_v + m_b * g - m_b * a_b_v;
raw_body_pitch = T_p_l + T_p_r + m_b * g * l_b * sin(theta_b + theta_b0) - I_b * ddtheta_b;

raw_left_leg_h = -F_l_to_b_h + F_wl_to_l_h - m_l * a_l_h;
raw_left_leg_v = -F_l_to_b_v + F_wl_to_l_v + m_l * g - m_l * a_l_v;
raw_left_leg_pitch = (F_wl_to_l_v * l_l_d + F_l_to_b_v * left_com_to_hip) * sin(theta_l) ...
    - (F_wl_to_l_h * l_l_d + F_l_to_b_h * left_com_to_hip) * cos(theta_l) ...
    + T_motor_stator_to_left_leg + T_p_l - I_l * ddtheta_l;

raw_right_leg_h = -F_r_to_b_h + F_wr_to_r_h - m_r * a_r_h;
raw_right_leg_v = -F_r_to_b_v + F_wr_to_r_v + m_r * g - m_r * a_r_v;
raw_right_leg_pitch = (F_wr_to_r_v * l_r_d + F_r_to_b_v * right_com_to_hip) * sin(theta_r) ...
    - (F_wr_to_r_h * l_r_d + F_r_to_b_h * right_com_to_hip) * cos(theta_r) ...
    + T_motor_stator_to_right_leg + T_p_r - I_r * ddtheta_r;

raw_left_wheel_h = -F_wl_to_l_h + F_g_to_wl_h - m_wl * a_wl_h;
raw_left_wheel_v = -F_wl_to_l_v + F_g_to_wl_v + m_wl * g - m_wl * a_wl_v;
raw_left_wheel_roll = -T_motor_stator_to_left_leg - F_g_to_wl_h * wheel_radius - I_wl * ddtheta_wl;

raw_right_wheel_h = -F_wr_to_r_h + F_g_to_wr_h - m_wr * a_wr_h;
raw_right_wheel_v = -F_wr_to_r_v + F_g_to_wr_v + m_wr * g - m_wr * a_wr_v;
raw_right_wheel_roll = -T_motor_stator_to_right_leg - F_g_to_wr_h * wheel_radius - I_wr * ddtheta_wr;

raw_yaw = (F_g_to_wr_h - F_g_to_wl_h) * half_track - I_yaw * ddphi;
raw_equal_normal = F_g_to_wl_v - F_g_to_wr_v;

raw_equations = [raw_body_h; raw_body_v; raw_body_pitch; ...
    raw_left_leg_h; raw_left_leg_v; raw_left_leg_pitch; ...
    raw_right_leg_h; raw_right_leg_v; raw_right_leg_pitch; ...
    raw_left_wheel_h; raw_left_wheel_v; raw_left_wheel_roll; ...
    raw_right_wheel_h; raw_right_wheel_v; raw_right_wheel_roll; ...
    raw_yaw; raw_equal_normal];

internal_forces = [F_l_to_b_h; F_l_to_b_v; F_r_to_b_h; F_r_to_b_v; ...
    F_wl_to_l_h; F_wl_to_l_v; F_wr_to_r_h; F_wr_to_r_v; ...
    F_g_to_wl_h; F_g_to_wl_v; F_g_to_wr_h; F_g_to_wr_v];

fprintf('  17 条原始方程，12 个接触内力；剩余 5 条为广义坐标方程。\n\n');

%% ========================================
%  Step 3: 左右独立纯轮式 s 运动学约束
%  ========================================

fprintf('Step 3: 代入左右独立的纯轮式 s 运动学约束...\n');

q_l = l_l * sin(theta_l);
q_r = l_r * sin(theta_r);
q_dd_l = l_l * cos(theta_l) * ddtheta_l - l_l * sin(theta_l) * dtheta_l ^ 2;
q_dd_r = l_r * cos(theta_r) * ddtheta_r - l_r * sin(theta_r) * dtheta_r ^ 2;

% R*theta_w,L = s-b*phi+(q_R-q_L)/2，R*theta_w,R = s+b*phi+(q_L-q_R)/2。
wheel_angular_acceleration_left = ...
    (dds - half_track * ddphi + (q_dd_r - q_dd_l) / 2) / wheel_radius;
wheel_angular_acceleration_right = ...
    (dds + half_track * ddphi + (q_dd_l - q_dd_r) / 2) / wheel_radius;

% s 是左右轮端平均滚动坐标，不是机身髋点坐标。
% 由 x_b = s + (q_L + q_R) / 2，机身水平加速度必须保留左右腿的 q_dd 项。
body_horizontal_acceleration = dds + (q_dd_l + q_dd_r) / 2;
body_vertical_acceleration = -(l_l * sin(theta_l) * ddtheta_l + l_r * sin(theta_r) * ddtheta_r) / 2 ...
    - (l_l * cos(theta_l) * dtheta_l ^ 2 + l_r * cos(theta_r) * dtheta_r ^ 2) / 2;

kinematic_substitution_symbols = [ddtheta_wl; ddtheta_wr; a_wl_h; a_wl_v; a_wr_h; a_wr_v; ...
    a_l_h; a_l_v; a_r_h; a_r_v; a_b_h; a_b_v];
kinematic_substitution_values = [wheel_angular_acceleration_left; wheel_angular_acceleration_right; ...
    wheel_radius * wheel_angular_acceleration_left; sym(0); ...
    wheel_radius * wheel_angular_acceleration_right; sym(0); ...
    wheel_radius * wheel_angular_acceleration_left + l_l_d * cos(theta_l) * ddtheta_l - ...
    l_l_d * sin(theta_l) * dtheta_l ^ 2; ...
    -l_l_d * sin(theta_l) * ddtheta_l - l_l_d * cos(theta_l) * dtheta_l ^ 2; ...
    wheel_radius * wheel_angular_acceleration_right + l_r_d * cos(theta_r) * ddtheta_r - ...
    l_r_d * sin(theta_r) * dtheta_r ^ 2; ...
    -l_r_d * sin(theta_r) * ddtheta_r - l_r_d * cos(theta_r) * dtheta_r ^ 2; ...
    body_horizontal_acceleration; body_vertical_acceleration];

kinematic_raw_equations = simplify(subs(raw_equations, kinematic_substitution_symbols, ...
    kinematic_substitution_values));

fprintf('  R*ddtheta_w,L = s_ddot-b*phi_ddot+(q_R_ddot-q_L_ddot)/2\n');
fprintf('  R*ddtheta_w,R = s_ddot+b*phi_ddot+(q_L_ddot-q_R_ddot)/2\n\n');

%% ========================================
%  Step 4: 自动消去接触内力
%  ========================================

fprintf('Step 4: 从原始方程自动消去 12 个接触内力...\n');

% 用平动方程和轮转动方程解出所有接触内力；其余方程自然成为五个广义残差。
% 法向力相等是解竖直接触内力所需的第六个约束；整体水平动量保留为广义方程。
force_equation_indices = [2, 4, 5, 7, 8, 10, 11, 12, 13, 14, 15, 17];
generalized_equation_indices = [1, 3, 6, 9, 16];
[force_matrix, force_rhs] = equationsToMatrix( ...
    kinematic_raw_equations(force_equation_indices) == sym(zeros(numel(force_equation_indices), 1)), ...
    internal_forces);
if rank(force_matrix) ~= numel(internal_forces)
    error('接触内力方程秩不足，不能完成自动消元。');
end
internal_force_solution = simplify(force_matrix \ force_rhs);
force_elimination_residual = simplify(force_matrix * internal_force_solution - force_rhs);
if ~all(arrayfun(@(value) isequal(value, sym(0)), force_elimination_residual))
    error('接触内力自动消元残差非零。');
end

equation = simplify(subs(kinematic_raw_equations(generalized_equation_indices), ...
    internal_forces, internal_force_solution));
generalized_equations = equation;

fprintf('  已由原始方程生成五条广义残差，不维护手写 eq1 至 eq5。\n\n');

%% ========================================
%  Step 5: 提取 M(q)*ddq = B(q)*u + g(q,dq)
%  ========================================

fprintf('Step 5: 提取 M、B 和 g...\n');

M = simplify(jacobian(equation, ddq));
B_raw = simplify(jacobian(equation, u));
B_control = -B_raw;
g_vector = simplify(-(equation - M * ddq - B_raw * u));
decomposition_residual = simplify(equation - (M * ddq - B_control * u - g_vector));
if ~all(arrayfun(@(value) isequal(value, sym(0)), decomposition_residual))
    error('M/B/g decomposition residual is not zero.');
end

fprintf('  M*ddq - B*u - g 与自动消元得到的五条方程严格一致。\n\n');

%% ========================================
%  Step 6: 导出数值函数
%  ========================================

fprintf('Step 6: 创建非线性模型函数...\n');

parameter_symbols = [m_b, m_l, m_r, m_wl, m_wr, I_b, I_l, I_r, I_wl, I_wr, I_yaw, ...
    l_l, l_r, l_l_d, l_r_d, l_b, wheel_radius, half_track, g, theta_l0, theta_r0, theta_b0];
M_func = matlabFunction(M, 'Vars', {x, parameter_symbols});
B_control_func = matlabFunction(B_control, 'Vars', {x, parameter_symbols});
g_func = matlabFunction(g_vector, 'Vars', {x, parameter_symbols});
force_solution_func = matlabFunction(internal_force_solution, 'Vars', {x, u, ddq, parameter_symbols});

save('double_closed_loop_symbolic_model.mat', ...
    'raw_equations', 'kinematic_raw_equations', 'internal_forces', 'force_equation_indices', ...
    'generalized_equation_indices', 'force_matrix', 'force_rhs', 'internal_force_solution', ...
    'force_elimination_residual', 'q_l', 'q_r', 'q_dd_l', 'q_dd_r', ...
    'T_motor_stator_to_left_leg', 'T_motor_stator_to_right_leg', ...
    'wheel_angular_acceleration_left', 'wheel_angular_acceleration_right', ...
    'kinematic_substitution_symbols', 'kinematic_substitution_values', ...
    'equation', 'generalized_equations', 'x', 'u', 'ddq', 'M', 'B_control', 'g_vector', ...
    'body_horizontal_acceleration', 'body_vertical_acceleration', 'parameter_symbols', ...
    'M_func', 'B_control_func', 'g_func', 'force_solution_func');

fprintf('  输出: double_closed_loop_symbolic_model.mat\n');
fprintf('========================================\n');
fprintf('符号模型构建完成\n');
fprintf('========================================\n');
end
