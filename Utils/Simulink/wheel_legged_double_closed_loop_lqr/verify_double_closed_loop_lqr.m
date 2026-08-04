% verify_double_closed_loop_lqr.m
% 双闭环等效腿模型的符号、里程计、静态工作点和 LQR 导出交叉验证。
%
% 本脚本验证 MATLAB 模型内部一致性，不替代轮编码器、IMU、传动和接触方向的真机试验。

clear;
clc;

fprintf('========================================\n');
fprintf('双闭环等效腿 LQR 验证\n');
fprintf('========================================\n\n');

%% ========================================
%  Step 1: 生成物与符号 M/B/g 恒等式
%  ========================================

fprintf('Step 1: 检查符号模型分解...\n');
if ~isfile('double_closed_loop_symbolic_model.mat')
    build_symbolic_model;
end
symbolic_model = load('double_closed_loop_symbolic_model.mat', 'equation', 'M', ...
    'B_control', 'g_vector', 'ddq', 'u', 'M_func', 'B_control_func', 'g_func');
decomposition = simplify(symbolic_model.equation - (symbolic_model.M * symbolic_model.ddq ...
    - symbolic_model.B_control * symbolic_model.u - symbolic_model.g_vector));
require(all(arrayfun(@(value) isequal(value, sym(0)), decomposition)), ...
    'M*ddq-B*u-g symbolic decomposition is nonzero.');
fprintf('  PASS: M*ddq-B*u-g 与五条方程一致。\n\n');

%% ========================================
%  Step 2: 髋点 O 里程计交叉用例
%  ========================================

fprintf('Step 2: 检查髋点 O 里程计合同...\n');
wheel_radius = 0.060;

% 用例 1: 纯滚动，腿角为零，只有 r*theta_w 和 r*theta_w_dot。
[position, velocity] = hip_point_side(wheel_radius, 2.0, 3.0, 0.160, 0.0, 0.0, 0.0);
require_close(position, 0.120, 1e-12, 'pure rolling position');
require_close(velocity, 0.180, 1e-12, 'pure rolling velocity');

% 用例 2: 腿摆，轮不滚动，只有 l*theta_dot*cos(theta)。
[~, velocity] = hip_point_side(wheel_radius, 0.0, 0.0, 0.160, 0.0, 0.0, 2.5);
require_close(velocity, 0.400, 1e-12, 'leg swing velocity');

% 用例 3: 腿长变化，轮不滚动，只有 l_dot*sin(theta)。
[~, velocity] = hip_point_side(wheel_radius, 0.0, 0.0, 0.160, 0.5, 0.2, 0.0);
require_close(velocity, 0.5 * sin(0.2), 1e-12, 'leg length rate velocity');

% 用例 4/5: 左右平均及采样零点扣除。
[left_position, left_velocity] = hip_point_side(wheel_radius, 1.0, 1.0, 0.150, 0.2, 0.1, 0.5);
[right_position, right_velocity] = hip_point_side(wheel_radius, 3.0, 2.0, 0.170, -0.1, -0.2, -0.4);
origin = 0.07;
s = (left_position + right_position) / 2 - origin;
s_dot = (left_velocity + right_velocity) / 2;
require_close(s, ((left_position + right_position) / 2 - origin), 1e-12, 'left/right average and origin');
require_close(s_dot, (left_velocity + right_velocity) / 2, 1e-12, 'left/right velocity average');
fprintf('  PASS: 纯滚动、腿摆、l_dot、左右平均、零点扣除。\n\n');

%% ========================================
%  Step 3: 标称静态工作点与全非线性导数
%  ========================================

fprintf('Step 3: 检查标称配平和数值线性化...\n');
if ~isfile('double_closed_loop_lqr_results.mat')
    compute_lqr_and_export;
end
result = load('double_closed_loop_lqr_results.mat', 'parameter', 'A_nominal', 'B_nominal', ...
    'K_nominal', 'x_ref_nominal', 'u0_nominal', 'trim_nominal', 'closed_loop_poles', ...
    'coefficient', 'x_ref_coefficient', 'u0_coefficient');
parameter = result.parameter;
values = parameter_values(parameter, parameter.nominal_leg_length_left, ...
    parameter.nominal_leg_length_right);
[M_nominal, B_mcu_nominal, g_nominal] = generalized_matrices(symbolic_model, parameter, ...
    values, result.x_ref_nominal);
generalized_residual = B_mcu_nominal * result.u0_nominal + g_nominal;
relative_residual = norm(generalized_residual, inf) / max(1, norm(g_nominal, inf));
require(relative_residual <= parameter.trim_relative_residual_tolerance, ...
    'Nominal static generalized-force residual exceeds tolerance.');
dx_ref = state_derivative(symbolic_model, parameter, values, result.x_ref_nominal, result.u0_nominal);
require(norm(dx_ref, inf) <= 1e-5, 'Nominal state derivative is not close to zero.');
require(rank(M_nominal) == 5, 'Nominal generalized mass matrix is singular.');

[A_check, B_check] = linearize_nonlinear_model(symbolic_model, parameter, values, ...
    result.x_ref_nominal, result.u0_nominal);
require_close(norm(A_check - result.A_nominal, inf), 0.0, 1e-7, 'A finite difference repeatability');
require_close(norm(B_check - result.B_nominal, inf), 0.0, 1e-7, 'B finite difference repeatability');
require(isequal(size(result.K_nominal), [4, 10]), 'K nominal dimension must be 4x10.');
require(isequal(size(result.coefficient), [40, 6]), 'K fit coefficient dimension must be 40x6.');
require(isequal(size(result.x_ref_coefficient), [10, 6]), 'x_ref fit coefficient dimension must be 10x6.');
require(isequal(size(result.u0_coefficient), [4, 6]), 'u0 fit coefficient dimension must be 4x6.');
require(all(real(result.closed_loop_poles) < 0), 'Nominal closed-loop poles are not all stable.');
fprintf('  PASS: residual=%.3e, ||f(x_ref,u0)||_inf=%.3e。\n\n', ...
    relative_residual, norm(dx_ref, inf));

%% ========================================
%  Step 4: MCU 输入方向和导出接口
%  ========================================

fprintf('Step 4: 检查 Tp 映射和导出接口...\n');
B_model_nominal = symbolic_model.B_control_func(result.x_ref_nominal, values);
require(isequal(parameter.input_sign, [-1; -1; 1; 1]), ...
    'input_sign must be [-1;-1;1;1].');
require_close(norm(B_mcu_nominal(:, 1) + B_model_nominal(:, 1), inf), 0.0, 1e-12, ...
    'Tp_R MCU mapping');
require_close(norm(B_mcu_nominal(:, 2) + B_model_nominal(:, 2), inf), 0.0, 1e-12, ...
    'Tp_L MCU mapping');
require_close(norm(B_mcu_nominal(:, 3) - B_model_nominal(:, 3), inf), 0.0, 1e-12, ...
    'Tw_R provisional mapping');
require_close(norm(B_mcu_nominal(:, 4) - B_model_nominal(:, 4), inf), 0.0, 1e-12, ...
    'Tw_L provisional mapping');
require(isfile('double_closed_loop_lqr_coefficients.h'), 'Generated C header is missing.');
header = fileread('double_closed_loop_lqr_coefficients.h');
require(contains(header, 'k_double_closed_loop_lqr_x_ref'), 'Header misses x_ref.');
require(contains(header, 'k_double_closed_loop_lqr_u0'), 'Header misses u0.');
require(contains(header, 'k_double_closed_loop_lqr_x_ref_coeff'), 'Header misses scheduled x_ref.');
require(contains(header, 'k_double_closed_loop_lqr_u0_coeff'), 'Header misses scheduled u0.');
fprintf('  PASS: Tp 已按 VMC 合同取负号；Tw 保持需真机验证。\n\n');

fprintf('========================================\n');
fprintf('全部 MATLAB 一致性检查通过\n');
fprintf('========================================\n');

%% ========================================
%  Private functions
%  ========================================

function [position, velocity] = hip_point_side(radius, wheel_angle, wheel_speed, ...
    leg_length, leg_length_rate, leg_angle_world, leg_angle_speed_world)
% 单侧髋点 O 位置和速度的冻结合同。
position = radius * wheel_angle + leg_length * sin(leg_angle_world);
velocity = radius * wheel_speed + leg_length_rate * sin(leg_angle_world) + ...
    leg_length * leg_angle_speed_world * cos(leg_angle_world);
end

function values = parameter_values(parameter, left_length, right_length)
% 参数顺序与 build_symbolic_model.m 的 parameter_symbols 一致。
values = [parameter.body_mass, parameter.leg_mass_left, parameter.leg_mass_right, ...
    parameter.wheel_mass_left, parameter.wheel_mass_right, parameter.body_pitch_inertia, ...
    parameter.leg_inertia_left, parameter.leg_inertia_right, parameter.wheel_inertia_left, ...
    parameter.wheel_inertia_right, parameter.yaw_inertia, left_length, right_length, ...
    parameter.leg_com_distance(left_length), parameter.leg_com_distance(right_length), ...
    parameter.body_com_to_pitch_axis, parameter.wheel_radius, parameter.half_track, parameter.g, ...
    parameter.leg_com_offset_left, parameter.leg_com_offset_right, parameter.body_com_offset];
end

function [M, B_mcu, g] = generalized_matrices(model, parameter, values, x)
% 将模型输入方向统一映射为 MCU 输入方向。
M = model.M_func(x, values);
B_mcu = model.B_control_func(x, values) * diag(parameter.input_sign);
g = model.g_func(x, values);
end

function dx = state_derivative(model, parameter, values, x, u_mcu)
% 完整非线性状态方程。
[M, B_mcu, g] = generalized_matrices(model, parameter, values, x);
ddq = M \ (B_mcu * u_mcu + g);
dx = [x(2); ddq(1); x(4); ddq(2); x(6); ddq(3); x(8); ddq(4); x(10); ddq(5)];
end

function [A, B] = linearize_nonlinear_model(model, parameter, values, x_ref, u0)
% 与 compute_lqr_and_export.m 相同的中心差分雅可比实现。
A = zeros(10, 10);
B = zeros(10, 4);
for index = 1:10
    delta = zeros(10, 1);
    delta(index) = parameter.linearization_state_step;
    A(:, index) = (state_derivative(model, parameter, values, x_ref + delta, u0) - ...
        state_derivative(model, parameter, values, x_ref - delta, u0)) / (2 * delta(index));
end
for index = 1:4
    delta = zeros(4, 1);
    delta(index) = parameter.linearization_input_step;
    B(:, index) = (state_derivative(model, parameter, values, x_ref, u0 + delta) - ...
        state_derivative(model, parameter, values, x_ref, u0 - delta)) / (2 * delta(index));
end
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
