% verify_double_closed_loop_lqr.m
% 双闭环等效腿模型的原始方程、运动学、Tw 合同、静态工作点和 LQR 导出交叉验证。
%
% 本脚本只验证 MATLAB 模型和影子 LQR 常量，不替代轮编码器、IMU、传动和轮毂方向的真机试验。

clear;
clc;

fprintf('========================================\n');
fprintf('双闭环等效腿 LQR 验证\n');
fprintf('========================================\n\n');

%% ========================================
%  Step 1: 原始方程自动消元与 M/B/g 恒等式
%  ========================================

fprintf('Step 1: 检查原始方程自动消元和 M/B/g 分解...\n');
build_symbolic_model;
symbolic_model = load('double_closed_loop_symbolic_model.mat', 'raw_equations', ...
    'kinematic_raw_equations', 'internal_forces', 'force_matrix', 'force_rhs', ...
    'internal_force_solution', 'force_elimination_residual', 'equation', ...
    'generalized_equations', 'M', 'B_control', 'g_vector', 'ddq', 'u', ...
    'M_func', 'B_control_func', 'g_func', 'q_dd_l', 'q_dd_r', ...
    'wheel_angular_acceleration_left', 'wheel_angular_acceleration_right');
require(numel(symbolic_model.raw_equations) == 17, '原始方程数量必须为 17。');
require(numel(symbolic_model.internal_forces) == 12, '待消去接触内力数量必须为 12。');
require(isequal(size(symbolic_model.generalized_equations), [5, 1]), '广义方程必须为五条。');
require(all(arrayfun(@(value) isequal(value, sym(0)), symbolic_model.force_elimination_residual)), ...
    '原始方程的接触内力消元残差非零。');
require(all(arrayfun(@(value) isequal(value, sym(0)), ...
    simplify(symbolic_model.equation - symbolic_model.generalized_equations))), ...
    '保存的广义方程与模型方程不一致。');
decomposition = simplify(symbolic_model.equation - (symbolic_model.M * symbolic_model.ddq ...
    - symbolic_model.B_control * symbolic_model.u - symbolic_model.g_vector));
require(all(arrayfun(@(value) isequal(value, sym(0)), decomposition)), ...
    'M*ddq-B*u-g symbolic decomposition is nonzero.');
fprintf('  PASS: 17 条原始方程 -> 消去 12 个内力 -> 5 条广义方程，且 M/B/g 一致。\n\n');

%% ========================================
%  Step 2: 左右独立机身 s_b 约束及 (2.5)--(2.8) 导数
%  ========================================

fprintf('Step 2: 检查左右独立机身 s_b 约束和一、二阶导数...\n');
syms wheel_radius half_track s phi theta_wl theta_wr theta_l theta_r real
syms ds dphi dtheta_wl dtheta_wr dtheta_l dtheta_r real
syms dds ddphi ddtheta_wl ddtheta_wr ddtheta_l ddtheta_r real
syms l_l l_r real

q_l = l_l * sin(theta_l);
q_r = l_r * sin(theta_r);
q_dot_l = l_l * cos(theta_l) * dtheta_l;
q_dot_r = l_r * cos(theta_r) * dtheta_r;
q_dd_l = l_l * cos(theta_l) * ddtheta_l - l_l * sin(theta_l) * dtheta_l ^ 2;
q_dd_r = l_r * cos(theta_r) * ddtheta_r - l_r * sin(theta_r) * dtheta_r ^ 2;

left_constraint = wheel_radius * theta_wl - (s - half_track * phi - q_l);
right_constraint = wheel_radius * theta_wr - (s + half_track * phi - q_r);
phi_from_constraints = (wheel_radius * (-theta_wl + theta_wr) - q_l + q_r) / (2 * half_track);
s_from_constraints = (wheel_radius * theta_wl + q_l + wheel_radius * theta_wr + q_r) / 2;
require(isequal(simplify(phi - phi_from_constraints - ...
    (left_constraint - right_constraint) / (2 * half_track)), sym(0)), ...
    '(2.5)/(2.6) 到 (2.7) 的符号消元错误。');
require(isequal(simplify(s - s_from_constraints + (left_constraint + right_constraint) / 2), sym(0)), ...
    '(2.5)/(2.6) 到 (2.8) 的符号消元错误。');

phi_dot_from_constraints = (wheel_radius * (-dtheta_wl + dtheta_wr) - q_dot_l + q_dot_r) / ...
    (2 * half_track);
phi_dd_from_constraints = (wheel_radius * (-ddtheta_wl + ddtheta_wr) - q_dd_l + q_dd_r) / ...
    (2 * half_track);
s_dot_from_constraints = (wheel_radius * dtheta_wl + q_dot_l + wheel_radius * dtheta_wr + q_dot_r) / 2;
s_dd_from_constraints = (wheel_radius * ddtheta_wl + q_dd_l + wheel_radius * ddtheta_wr + q_dd_r) / 2;
require(isequal(simplify(phi_dot_from_constraints - ...
    (wheel_radius * (-dtheta_wl + dtheta_wr) - l_l * cos(theta_l) * dtheta_l + ...
    l_r * cos(theta_r) * dtheta_r) / (2 * half_track)), sym(0)), ...
    '(2.7) 一阶导数错误。');
require(isequal(simplify(phi_dd_from_constraints - ...
    (wheel_radius * (-ddtheta_wl + ddtheta_wr) - l_l * cos(theta_l) * ddtheta_l + ...
    l_l * sin(theta_l) * dtheta_l ^ 2 + l_r * cos(theta_r) * ddtheta_r - ...
    l_r * sin(theta_r) * dtheta_r ^ 2) / (2 * half_track)), sym(0)), ...
    '(2.7) 二阶导数错误。');
require(isequal(simplify(s_dot_from_constraints - ...
    (wheel_radius * (dtheta_wl + dtheta_wr) + l_l * cos(theta_l) * dtheta_l + ...
    l_r * cos(theta_r) * dtheta_r) / 2), sym(0)), '(2.8) 一阶导数错误。');
require(isequal(simplify(s_dd_from_constraints - ...
    (wheel_radius * (ddtheta_wl + ddtheta_wr) + q_dd_l + q_dd_r) / 2), sym(0)), ...
    '(2.8) 二阶导数错误。');

% 对真实 builder 的左右轮角加速度逐项检查，禁止回退为左右腿平均项。
builder_yaw_kinematic_residual = simplify((wheel_radius / (2 * half_track)) * ...
    (-symbolic_model.wheel_angular_acceleration_left + symbolic_model.wheel_angular_acceleration_right) ...
    - symbolic_model.q_dd_l / (2 * half_track) + symbolic_model.q_dd_r / (2 * half_track) - ddphi);
require(isequal(builder_yaw_kinematic_residual, sym(0)), 'builder 中的左右独立 Yaw 约束错误。');
require(~isequal(simplify(diff(symbolic_model.wheel_angular_acceleration_left, ddtheta_l)), sym(0)), ...
    '左轮约束未保留 q_L 二阶导数。');
require(isequal(simplify(diff(symbolic_model.wheel_angular_acceleration_left, ddtheta_r)), sym(0)), ...
    '左轮约束错误引入 q_R 二阶导数。');
require(~isequal(simplify(diff(symbolic_model.wheel_angular_acceleration_right, ddtheta_r)), sym(0)), ...
    '右轮约束未保留 q_R 二阶导数。');
require(isequal(simplify(diff(symbolic_model.wheel_angular_acceleration_right, ddtheta_l)), sym(0)), ...
    '右轮约束错误引入 q_L 二阶导数。');
fprintf('  PASS: (2.5)/(2.6) 到 (2.7)/(2.8) 及一、二阶导数一致，Yaw 保留 q_R-q_L。\n\n');

%% ========================================
%  Step 3: 髋点 O 里程计数值交叉用例
%  ========================================

fprintf('Step 3: 检查髋点 O 里程计合同...\n');
radius = 0.060;
[position, velocity] = hip_point_side(radius, 2.0, 3.0, 0.160, 0.0, 0.0, 0.0);
require_close(position, 0.120, 1e-12, 'pure rolling position');
require_close(velocity, 0.180, 1e-12, 'pure rolling velocity');
[~, velocity] = hip_point_side(radius, 0.0, 0.0, 0.160, 0.0, 0.0, 2.5);
require_close(velocity, 0.400, 1e-12, 'leg swing velocity');
[~, velocity] = hip_point_side(radius, 0.0, 0.0, 0.160, 0.5, 0.2, 0.0);
require_close(velocity, 0.5 * sin(0.2), 1e-12, 'leg length rate velocity');
fprintf('  PASS: 纯滚动、腿摆和 l_dot 项均符合髋点 O 定义。\n\n');

%% ========================================
%  Step 4: 静态工作点、Tw 合同和 LQR
%  ========================================

fprintf('Step 4: 检查静态配平、Tw 方向和 LQR 导出...\n');
compute_lqr_and_export;
result = load('double_closed_loop_lqr_results.mat', 'parameter', 'A_nominal', 'B_nominal', ...
    'K_nominal', 'x_ref_nominal', 'u0_nominal', 'trim_nominal', 'closed_loop_poles', ...
    'controllability_rank');
parameter = result.parameter;
require(isequal(parameter.input_sign, [-1; -1; +1; +1]), ...
    'input_sign must be [-1;-1;+1;+1].');
require(isequal(size(parameter.bryson_state_tuning), [10, 2]), 'Bryson state table must be 10x2.');
require(isequal(size(parameter.bryson_input_tuning), [4, 2]), 'Bryson input table must be 4x2.');
values = parameter_values(parameter, parameter.nominal_leg_length_left, parameter.nominal_leg_length_right);
[M_nominal, B_mcu_nominal, g_nominal] = generalized_matrices(symbolic_model, parameter, values, ...
    result.x_ref_nominal);
generalized_residual = B_mcu_nominal * result.u0_nominal + g_nominal;
relative_residual = norm(generalized_residual, inf) / max(1, norm(g_nominal, inf));
require(relative_residual <= parameter.trim_relative_residual_tolerance, ...
    'Nominal static generalized-force residual exceeds tolerance.');
require(rank(M_nominal) == 5, 'Nominal generalized mass matrix is singular.');
require(isequal(size(result.K_nominal), [4, 10]), 'K nominal dimension must be 4x10.');
require(all(real(result.closed_loop_poles) < 0), 'Nominal closed-loop poles are not all stable.');

B_model_nominal = symbolic_model.B_control_func(result.x_ref_nominal, values);
require_close(norm(B_mcu_nominal(:, 1) + B_model_nominal(:, 1), inf), 0.0, 1e-12, 'Tp_R mapping');
require_close(norm(B_mcu_nominal(:, 2) + B_model_nominal(:, 2), inf), 0.0, 1e-12, 'Tp_L mapping');
require_close(norm(B_mcu_nominal(:, 3) - B_model_nominal(:, 3), inf), 0.0, 1e-12, 'Tw_R mapping');
require_close(norm(B_mcu_nominal(:, 4) - B_model_nominal(:, 4), inf), 0.0, 1e-12, 'Tw_L mapping');

input_acceleration = M_nominal \ B_mcu_nominal;
right_wheel_acceleration = wheel_acceleration_right(input_acceleration(:, 3), parameter, ...
    result.x_ref_nominal);
left_wheel_acceleration = wheel_acceleration_left(input_acceleration(:, 4), parameter, ...
    result.x_ref_nominal);
require(right_wheel_acceleration > 0.0 && input_acceleration(2, 3) > 0.0, ...
    'Tw_R > 0 must make the right wheel roll forward and phi_ddot > 0.');
require(left_wheel_acceleration > 0.0 && input_acceleration(2, 4) < 0.0, ...
    'Tw_L > 0 must make the left wheel roll forward and phi_ddot < 0.');
fprintf('  Tw_R>0: s_ddot=%+.6f, right wheel_ddot=%+.6f, phi_ddot=%+.6f\n', ...
    input_acceleration(1, 3), right_wheel_acceleration, input_acceleration(2, 3));
fprintf('  Tw_L>0: s_ddot=%+.6f, left wheel_ddot=%+.6f, phi_ddot=%+.6f\n', ...
    input_acceleration(1, 4), left_wheel_acceleration, input_acceleration(2, 4));

state_error_s = zeros(10, 1);
state_error_s(1) = 1.0;
state_error_phi = zeros(10, 1);
state_error_phi(3) = 1.0;
feedback_s = -result.K_nominal * state_error_s;
feedback_phi = -result.K_nominal * state_error_phi;
fprintf('  s>0 时 -Kx 的 [Tp_R Tp_L Tw_R Tw_L] = [%+.6f %+.6f %+.6f %+.6f]\n', feedback_s);
fprintf('  phi>0 时 -Kx 的 [Tp_R Tp_L Tw_R Tw_L] = [%+.6f %+.6f %+.6f %+.6f]\n', feedback_phi);
restoring_acceleration_s = input_acceleration * feedback_s;
restoring_acceleration_phi = input_acceleration * feedback_phi;
require(restoring_acceleration_s(1) < 0.0, ...
    's > 0 must receive a restoring total s_ddot from the complete four-input feedback.');
require(restoring_acceleration_phi(2) < 0.0, ...
    'phi > 0 must receive a restoring total phi_ddot from the complete four-input feedback.');

require(isfile('double_closed_loop_lqr_coefficients.h'), 'Generated C header is missing.');
header = fileread('double_closed_loop_lqr_coefficients.h');
require(contains(header, 'k_double_closed_loop_lqr_input_sign[4] = {-1.0f, -1.0f, 1.0f, 1.0f}'), ...
    'Generated header has stale input signs.');
fprintf('  PASS: residual=%.3e；Tw 正方向、-Kx 贡献和导出头文件均一致。\n\n', relative_residual);

fprintf('========================================\n');
fprintf('全部 MATLAB 一致性检查通过\n');
fprintf('========================================\n');

%% ========================================
%  Private functions
%  ========================================

function [position, velocity] = hip_point_side(radius, wheel_angle, wheel_speed, ...
    leg_length, leg_length_rate, leg_angle_longitudinal, leg_angle_speed_longitudinal)
% 单侧髋点 O 的车体纵向二维位置和速度。
position = radius * wheel_angle + leg_length * sin(leg_angle_longitudinal);
velocity = radius * wheel_speed + leg_length_rate * sin(leg_angle_longitudinal) + ...
    leg_length * leg_angle_speed_longitudinal * cos(leg_angle_longitudinal);
end

function acceleration = wheel_acceleration_left(generalized_acceleration, parameter, x_ref)
% 由左侧独立髋点约束恢复左轮角加速度，正值表示左轮向前滚动。
q_dd = parameter.nominal_leg_length_left * cos(x_ref(5)) * generalized_acceleration(3);
acceleration = (generalized_acceleration(1) - parameter.half_track * generalized_acceleration(2) - q_dd) / ...
    parameter.wheel_radius;
end

function acceleration = wheel_acceleration_right(generalized_acceleration, parameter, x_ref)
% 由右侧独立髋点约束恢复右轮角加速度，正值表示右轮向前滚动。
q_dd = parameter.nominal_leg_length_right * cos(x_ref(7)) * generalized_acceleration(4);
acceleration = (generalized_acceleration(1) + parameter.half_track * generalized_acceleration(2) - q_dd) / ...
    parameter.wheel_radius;
end

function values = parameter_values(parameter, left_length, right_length)
% 参数顺序与 build_symbolic_model.m 的 parameter_symbols 严格一致。
values = [parameter.body_mass, parameter.leg_mass_left, parameter.leg_mass_right, ...
    parameter.wheel_mass_left, parameter.wheel_mass_right, parameter.body_pitch_inertia, ...
    parameter.leg_inertia_left, parameter.leg_inertia_right, parameter.wheel_inertia_left, ...
    parameter.wheel_inertia_right, parameter.yaw_inertia, left_length, right_length, ...
    parameter.leg_com_distance(left_length), parameter.leg_com_distance(right_length), ...
    parameter.body_com_to_pitch_axis, parameter.wheel_radius, parameter.half_track, parameter.g, ...
    parameter.leg_com_offset_left, parameter.leg_com_offset_right, parameter.body_com_offset];
end

function [M, B_mcu, g] = generalized_matrices(model, parameter, values, x)
% 将模型输入方向统一映射为 MCU 影子 LQR 的输入方向。
M = model.M_func(x, values);
B_mcu = model.B_control_func(x, values) * diag(parameter.input_sign);
g = model.g_func(x, values);
end

function require(condition, message)
% 统一错误接口，便于定位具体检查项。
if ~condition
    error('Verification failed: %s', message);
end
end

function require_close(actual, expected, tolerance, name)
% 检查数值误差是否在给定绝对容差内。
if abs(actual - expected) > tolerance
    error('Verification failed: %s, actual %.16g, expected %.16g.', name, actual, expected);
end
end
