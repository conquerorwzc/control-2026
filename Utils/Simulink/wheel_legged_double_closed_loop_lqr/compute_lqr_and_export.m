function compute_lqr_and_export()
% compute_lqr_and_export.m
% 双闭环等效虚拟腿模型: 固定工作点的静态配平、线性化、LQR 与 C 导出。
%
% 控制合同:
%   u_mcu = u0 - K * (x - x_ref)
%
% 动力学边界:
%   测量端 s_dot 保留 l_dot 项；单个 LQR 工作点采用固定腿长模型，
%   不把 l_dot、l_ddot 作为动力学状态或输入。

clc;

fprintf('========================================\n');
fprintf('双闭环等效腿 LQR 计算与导出\n');
fprintf('========================================\n\n');

%% ========================================
%  Step 1: 读取参数和非线性符号模型
%  ========================================

fprintf('Step 1: 读取模型合同与参数...\n');
parameter = double_closed_loop_parameters();
% 每次导出前重建符号模型，避免脚本或 Tw 合同变更后误用旧 MAT 文件。
build_symbolic_model;
model = load('double_closed_loop_symbolic_model.mat', ...
    'M_func', 'B_control_func', 'g_func');

fprintf('  状态顺序: [%s]\n', strjoin(parameter.state_order, ', '));
fprintf('  输入顺序: [%s]\n', strjoin(parameter.input_order, ', '));
fprintf('  MCU 输入符号: [%g, %g, %g, %g]\n', parameter.input_sign);
fprintf('  Tp_R/Tp_L 已按 VMC 合同取负号；Tw_R/Tw_L 正值定义为对应轮向前，实际电机方向仍需标定。\n\n');
fprintf('  Bryson 状态调参表 [e_max, q] =\n'); disp(parameter.bryson_state_tuning);
fprintf('  Bryson 输入调参表 [du_max, r] =\n'); disp(parameter.bryson_input_tuning);
fprintf('  Q 对角线 = '); disp(diag(parameter.Q).');
fprintf('  R 对角线 = '); disp(diag(parameter.R).');
fprintf('\n');

%% ========================================
%  Step 2: 固定工作点、可控性与闭环极点
%  ========================================

fprintf('Step 2: 计算固定 0.160 m 工作点详情...\n');
nominal_values = parameter_values(parameter, parameter.nominal_leg_length_left, ...
    parameter.nominal_leg_length_right);
[x_ref_nominal, u0_nominal, trim_nominal] = solve_static_trim(model, parameter, nominal_values);
if trim_nominal.relative_residual > parameter.trim_relative_residual_tolerance
    error('Nominal static trim failed. K is not exported.');
end
[A_nominal, B_nominal] = linearize_nonlinear_model(model, parameter, nominal_values, ...
    x_ref_nominal, u0_nominal);
K_nominal = continuous_lqr(A_nominal, B_nominal, parameter.Q, parameter.R);
closed_loop_poles = eig(A_nominal - B_nominal * K_nominal);
controllability_rank = rank(controllability_matrix(A_nominal, B_nominal));

fprintf('  x_ref^T = '); disp(x_ref_nominal.');
fprintf('  u0^T    = '); disp(u0_nominal.');
fprintf('  静态配平相对残差: %.3e\n', trim_nominal.relative_residual);
fprintf('  A_nominal:\n'); disp(A_nominal);
fprintf('  B_nominal (MCU 输入方向):\n'); disp(B_nominal);
fprintf('  可控性 rank: %d / %d\n', controllability_rank, size(A_nominal, 1));
fprintf('  Q:\n'); disp(parameter.Q);
fprintf('  R:\n'); disp(parameter.R);
fprintf('  标称 K:\n'); disp(K_nominal);
fprintf('  闭环极点:\n'); disp(closed_loop_poles.');
if any(real(closed_loop_poles) >= 0)
    error('Nominal closed-loop system is unstable. K is not exported.');
end
fprintf('\n');

%% ========================================
%  Step 3: 保存与导出
%  ========================================

fprintf('Step 3: 导出 MAT 与 C 初始化器...\n');
export_lqr_header('double_closed_loop_lqr_coefficients.h', parameter, K_nominal, x_ref_nominal, ...
    u0_nominal, trim_nominal.relative_residual);
save('double_closed_loop_lqr_results.mat', 'parameter', 'A_nominal', 'B_nominal', 'K_nominal', ...
    'x_ref_nominal', 'u0_nominal', 'trim_nominal', 'closed_loop_poles', 'controllability_rank');
fprintf('  输出: double_closed_loop_lqr_coefficients.h\n');
fprintf('  输出: double_closed_loop_lqr_results.mat\n');
fprintf('========================================\n');
fprintf('LQR 计算与导出完成\n');
fprintf('========================================\n');

end

%% ========================================
%  Private functions
%  ========================================

function values = parameter_values(parameter, left_length, right_length)
% 将结构体参数按 build_symbolic_model.m 的 parameter_symbols 顺序打包。
values = [parameter.body_mass, parameter.leg_mass_left, parameter.leg_mass_right, ...
    parameter.wheel_mass_left, parameter.wheel_mass_right, parameter.body_pitch_inertia, ...
    parameter.leg_inertia_left, parameter.leg_inertia_right, parameter.wheel_inertia_left, ...
    parameter.wheel_inertia_right, parameter.yaw_inertia, left_length, right_length, ...
    parameter.leg_com_distance(left_length), parameter.leg_com_distance(right_length), ...
    parameter.body_com_to_pitch_axis, parameter.wheel_radius, parameter.half_track, parameter.g, ...
    parameter.leg_com_offset_left, parameter.leg_com_offset_right, parameter.body_com_offset];
end

function [x_ref, u0, trim] = solve_static_trim(model, parameter, values)
% 固定 theta_b，优化左右虚拟腿角；每个候选角度用最小范数 u0 消除广义力。
initial_angles = [parameter.trim_initial_leg_angle; parameter.trim_initial_leg_angle];
options = optimset('Display', 'off', 'TolX', 1e-12, 'TolFun', 1e-18, ...
    'MaxIter', 2000, 'MaxFunEvals', 5000);
angles = fminsearch(@(candidate) trim_objective(candidate, model, parameter, values), ...
    initial_angles, options);
x_ref = [0; 0; 0; 0; angles(1); 0; angles(2); 0; ...
    parameter.trim_body_pitch_reference; 0];
[M, B_mcu, g] = generalized_matrices(model, parameter, values, x_ref);
u0 = -pinv(B_mcu) * g;
residual = B_mcu * u0 + g;
trim.residual = residual;
trim.relative_residual = norm(residual, inf) / max(1, norm(g, inf));
trim.state_derivative = state_derivative(model, parameter, values, x_ref, u0);
trim.matrix_rank = rank(B_mcu);
trim.M = M;
end

function cost = trim_objective(angles, model, parameter, values)
% 残差优先；其余项只在可实现解族中选取小角度、近对称、较小输入解。
x_candidate = [0; 0; 0; 0; angles(1); 0; angles(2); 0; ...
    parameter.trim_body_pitch_reference; 0];
[~, B_mcu, g] = generalized_matrices(model, parameter, values, x_candidate);
u_candidate = -pinv(B_mcu) * g;
residual = B_mcu * u_candidate + g;
cost = norm(residual, 2)^2 ...
    + parameter.trim_angle_regularization * (angles(1)^2 + angles(2)^2 + ...
    (angles(1) - angles(2))^2) ...
    + parameter.trim_input_regularization * norm(u_candidate, 2)^2;
end

function [A, B] = linearize_nonlinear_model(model, parameter, values, x_ref, u0)
% 用中心差分在线性工作点对完整 f(x,u) 求雅可比，包含 B(q)*u0 的姿态耦合。
state_count = numel(x_ref);
input_count = numel(u0);
A = zeros(state_count, state_count);
B = zeros(state_count, input_count);
for index = 1:state_count
    delta = zeros(state_count, 1);
    delta(index) = parameter.linearization_state_step;
    A(:, index) = (state_derivative(model, parameter, values, x_ref + delta, u0) - ...
        state_derivative(model, parameter, values, x_ref - delta, u0)) / (2 * delta(index));
end
for index = 1:input_count
    delta = zeros(input_count, 1);
    delta(index) = parameter.linearization_input_step;
    B(:, index) = (state_derivative(model, parameter, values, x_ref, u0 + delta) - ...
        state_derivative(model, parameter, values, x_ref, u0 - delta)) / (2 * delta(index));
end
end

function dx = state_derivative(model, parameter, values, x, u_mcu)
% 完整非线性状态方程: M(q) * ddq = B_mcu(q) * u_mcu + g(q,dq)。
[M, B_mcu, g] = generalized_matrices(model, parameter, values, x);
ddq = M \ (B_mcu * u_mcu + g);
dx = [x(2); ddq(1); x(4); ddq(2); x(6); ddq(3); ...
    x(8); ddq(4); x(10); ddq(5)];
end

function [M, B_mcu, g] = generalized_matrices(model, parameter, values, x)
% model 的 B_control 使用模型输入正方向；此处统一映射到 MCU 输入正方向。
M = model.M_func(x, values);
B_model = model.B_control_func(x, values);
g = model.g_func(x, values);
B_mcu = B_model * diag(parameter.input_sign);
end

function export_lqr_header(path, parameter, K, x_ref, u0, trim_residual)
% 导出直接可嵌入 MCU 的常量；不在本脚本接入或执行底盘闭环控制。
file_id = fopen(path, 'w');
if file_id < 0
    error('Cannot open %s for writing.', path);
end
fprintf(file_id, '#pragma once\n\n');
fprintf(file_id, '/* Generated by compute_lqr_and_export.m. */\n');
fprintf(file_id, '/* x: [s, s_dot, phi, phi_dot, theta_L, theta_L_dot, theta_R, theta_R_dot, theta_b, theta_b_dot]. */\n');
fprintf(file_id, '/* u_mcu: [Tp_R, Tp_L, Tw_R, Tw_L]. */\n');
fprintf(file_id, '/* u_mcu = u0 - K * (x - x_ref). */\n');
fprintf(file_id, '/* Tp signs are aligned with the current VMC contract. Positive Tw means the corresponding wheel rolls forward; H6215 command direction still requires hardware calibration. */\n\n');
fprintf(file_id, '/* Fixed 0.160 m design point. The current shadow LQR does not gate on leg length. */\n');
fprintf(file_id, 'static const float k_double_closed_loop_lqr_fixed_leg_length = %.9ef;\n', parameter.fixed_leg_length);
fprintf(file_id, '\n');
fprintf(file_id, 'static const float k_double_closed_loop_lqr_input_sign[4] = {%.1ff, %.1ff, %.1ff, %.1ff};\n', ...
    parameter.input_sign);
write_c_vector(file_id, 'k_double_closed_loop_lqr_x_ref', x_ref);
write_c_vector(file_id, 'k_double_closed_loop_lqr_u0', u0);
fprintf(file_id, 'static const float k_double_closed_loop_lqr_trim_relative_residual = %.9ef;\n\n', trim_residual);
fprintf(file_id, 'static const float k_double_closed_loop_lqr_nominal[4][10] = {\n');
for row = 1:4
    fprintf(file_id, '    {');
    fprintf(file_id, '%.9ef, ', K(row, 1:9));
    fprintf(file_id, '%.9ef}%s\n', K(row, 10), ternary(row < 4, ',', ''));
end
fprintf(file_id, '};\n');
fclose(file_id);
end

function write_c_vector(file_id, name, vector)
% 以 C 一维数组格式写入 x_ref 或 u0。
fprintf(file_id, 'static const float %s[%d] = {', name, numel(vector));
fprintf(file_id, '%.9ef, ', vector(1:end - 1));
fprintf(file_id, '%.9ef};\n', vector(end));
end

function value = ternary(condition, true_value, false_value)
% MATLAB 无内置 C 三元运算符；仅用于 C 初始化器末尾逗号。
if condition
    value = true_value;
else
    value = false_value;
end
end

function K = continuous_lqr(A, B, Q, R)
% 用 Hamiltonian 稳定不变子空间求连续 CARE，避免依赖 Control System Toolbox。
state_count = size(A, 1);
R_inverse_B_transpose = R \ B.';
hamiltonian = [A, -B * R_inverse_B_transpose; -Q, -A.'];
[vectors, schur_form] = schur(hamiltonian, 'complex');
eigenvalues = ordeig(schur_form);
stable = real(eigenvalues) < -1e-9;
if nnz(stable) ~= state_count
    error('CARE stable subspace has %d vectors; expected %d.', nnz(stable), state_count);
end
[vectors, ~] = ordschur(vectors, schur_form, stable);
upper = vectors(1:state_count, 1:state_count);
lower = vectors(state_count + 1:end, 1:state_count);
P = real(lower / upper);
P = (P + P.') / 2;
K = R_inverse_B_transpose * P;
end

function matrix = controllability_matrix(A, B)
% 构造 [B, A*B, ..., A^(n-1)*B]，避免依赖 Control System Toolbox 的 ctrb。
state_count = size(A, 1);
matrix = B;
power_times_b = B;
for index = 2:state_count
    power_times_b = A * power_times_b;
    matrix = [matrix, power_times_b]; %#ok<AGROW>
end
end
