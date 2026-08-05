function parameter = double_closed_loop_parameters()
% double_closed_loop_parameters.m
% 双闭环轮腿等效模型的数值参数、静态工作点和 LQR 配置。
%
% 状态顺序:
%   x = [s, s_dot, phi, phi_dot, theta_L, theta_L_dot,
%        theta_R, theta_R_dot, theta_b, theta_b_dot]^T
%
% 输入顺序:
%   u_mcu = [Tp_R, Tp_L, Tw_R, Tw_L]^T
%
% s 是左右轮端平均滚动得到的纯轮式纵向二维坐标，不包含腿长和腿角项。
% 髋点 O 里程计仍可作为固件诊断量，但不参与十维状态和 LQR。
% 单个 LQR 工作点仍采用固定腿长近似，忽略 l_dot、l_ddot 的动力学影响。

%% ========================================
%  Step 1: 状态与输入合同
%  ========================================

parameter.state_order = {'s', 's_dot', 'phi', 'phi_dot', 'theta_L', ...
    'theta_L_dot', 'theta_R', 'theta_R_dot', 'theta_b', 'theta_b_dot'};
parameter.model_input_order = {'T_lw_L', 'T_lw_R', 'T_bl_L', 'T_bl_R'};
parameter.input_order = {'Tp_R', 'Tp_L', 'Tw_R', 'Tw_L'};
% u_model = input_map_model_from_mcu * diag(input_sign) * u_mcu。
% 这里只做文章输入顺序到 MCU 输入顺序的置换；四个物理方向均同号。
parameter.input_map_model_from_mcu = [ ...
    0, 0, 0, 1; ... % T_lw,L = Tw_L
    0, 0, 1, 0; ... % T_lw,R = Tw_R
    0, 1, 0, 0; ... % T_bl,L = Tp_L
    1, 0, 0, 0; ... % T_bl,R = Tp_R
];

%% ========================================
%  Step 2: 重力与几何参数
%  ========================================

% g 是原文约定的正重力大小；竖直方程显式使用 -m*g。
parameter.g = 9.81;
parameter.wheel_radius = 0.060;
parameter.half_track = 0.304 / 2;

%% ========================================
%  Step 3: 机身、虚拟腿与轮参数
%  ========================================

% 当前调试采用外部供电，电池放在地上；1.183 kg 是双腿实际支撑的完整机身总成质量。
parameter.body_mass_without_battery = 1.183;
parameter.battery_mass = 0.0;
parameter.body_mass = parameter.body_mass_without_battery + parameter.battery_mass;
parameter.body_pitch_inertia = 0.0064;
% l_c：机身质心到等效髋点的距离。符号模型保留 SJTU 原式 (3.8) 的
% l_c 力矩项，但原文 (2.2)/(2.4) 在 l_c 非零时几何不闭合；必须先补全
% 机身运动学，不能只修改此参数。
parameter.body_com_to_hip_distance = 0.0;
parameter.yaw_inertia = 0.0845;

parameter.leg_mass_left = 0.206;
parameter.leg_mass_right = 0.206;
parameter.leg_inertia_left = 0.000962;
parameter.leg_inertia_right = 0.000962;

parameter.wheel_mass_left = 0.650;
parameter.wheel_mass_right = 0.650;
parameter.wheel_inertia_left = 0.000896;
parameter.wheel_inertia_right = 0.000896;

parameter.leg_com_distance = @(leg_length) 0.5 * leg_length;

assert(parameter.body_com_to_hip_distance == 0.0, ...
    '当前模型仅支持机身质心位于髋轴 O；非零偏距必须先补全机身运动学闭环。');

%% ========================================
%  Step 4: 固定腿长工作点与静态配平
%  ========================================

% 当前固定在双腿均为 0.170 m 的直立工作点，不做腿长调度或多项式拟合。
% MCU LQR 当前不按腿长门控；实测腿长仅用于变量窗口观察。
parameter.fixed_leg_length = 0.170;
parameter.nominal_leg_length_left = parameter.fixed_leg_length;
parameter.nominal_leg_length_right = parameter.fixed_leg_length;

% 自动配平保持机身 Pitch 为此参考值，并求左右虚拟腿静态角和 u0。
parameter.trim_body_pitch_reference = 0.0;
parameter.trim_initial_leg_angle = 0.0;
parameter.trim_relative_residual_tolerance = 1e-7;
parameter.trim_angle_regularization = 1e-8;
parameter.trim_input_regularization = 1e-10;

%% ========================================
%  Step 5: 模型到 MCU 的输入方向
%  ========================================

% 模型和固件 VMC 的正 Tp 均定义为机身施加给腿、使虚拟腿角增大的广义力矩；
% 正 Tw 均定义为使对应轮向前滚动的轮端驱动力矩，因此四个输入均不取反。
parameter.input_sign = [+1; +1; +1; +1];
parameter.input_sign_status = {'Tp_R 正值为机身对右腿的广义力矩', 'Tp_L 正值为机身对左腿的广义力矩', ...
    'Tw_R 正值定义为右轮向前，待实际输出标定', 'Tw_L 正值定义为左轮向前，待实际输出标定'};
assert(isequal(parameter.input_map_model_from_mcu * parameter.input_map_model_from_mcu.', eye(4)), ...
    '文章输入到 MCU 输入的映射必须是正交置换矩阵。');

%% ========================================
%  Step 6: Bryson 法 Q/R 权重
%  ========================================

% Bryson 法按 Q_ii=q_i/e_i,max^2 归一化状态误差。每行顺序严格对应
% state_order，第一列 e_max 是有物理单位的可接受误差，第二列 q 是无量纲
% 优先级倍数。边界不是机械硬限位；日常闭环调参优先修改 q，避免混淆物理边界。
parameter.bryson_state_tuning = [ ...
    0.10,        1.0;  % s：纯轮式纵向位移，单位 m。
    0.50,        1.0;  % s_dot：纯轮式纵向速度，单位 m/s。
    deg2rad(10), 1.0;  % phi：偏航角，单位 rad。
    1.00,        1.0;  % phi_dot：偏航角速度，单位 rad/s。
    deg2rad(10), 5.0;   % theta_L：左腿纵向平面角，单位 rad。
    2.00,        5.0;  % theta_L_dot：左腿角速度，单位 rad/s。
    deg2rad(10), 5.0;   % theta_R：右腿纵向平面角，单位 rad。
    2.00,        5.0;  % theta_R_dot：右腿角速度，单位 rad/s。
    deg2rad(5), 20.0;  % theta_b：机身俯仰角，单位 rad。
    1.50,        5.0;  % theta_b_dot：机身俯仰角速度，单位 rad/s。
];
parameter.bryson_state_error_limit = parameter.bryson_state_tuning(:, 1);
parameter.bryson_state_weight_multiplier = parameter.bryson_state_tuning(:, 2);

% 输入按 R_jj=r_j/delta_u_j,max^2 归一化。每行顺序严格对应 input_order，
% 第一列 delta_u_max 是相对静态配平输入 u0 的允许控制增量，单位 N*m；第二列
% r 是无量纲输入抑制倍数。delta_u_max 不是电机轴、VMC 或轮毂的实际输出限幅。
parameter.bryson_input_tuning = [ ...
    0.50, 1.0; % Tp_R：右腿虚拟摆动广义力矩，单位 N*m。
    0.50, 1.0; % Tp_L：左腿虚拟摆动广义力矩，单位 N*m。
    1.00, 15.0; % Tw_R：右轮广义力矩，单位 N*m。
    1.00, 15.0; % Tw_L：左轮广义力矩，单位 N*m。
];
parameter.bryson_input_delta_limit = parameter.bryson_input_tuning(:, 1);
parameter.bryson_input_weight_multiplier = parameter.bryson_input_tuning(:, 2);

assert(isequal(size(parameter.bryson_state_tuning), [10, 2]), ...
    'Bryson 状态调参表必须为 10x2。');
assert(isequal(size(parameter.bryson_input_tuning), [4, 2]), ...
    'Bryson 输入调参表必须为 4x2。');
assert(isequal(size(parameter.bryson_state_error_limit), [10, 1]), ...
    'Bryson 状态误差边界必须为 10x1。');
assert(isequal(size(parameter.bryson_state_weight_multiplier), [10, 1]), ...
    'Bryson 状态权重倍数必须为 10x1。');
assert(isequal(size(parameter.bryson_input_delta_limit), [4, 1]), ...
    'Bryson 输入增量边界必须为 4x1。');
assert(isequal(size(parameter.bryson_input_weight_multiplier), [4, 1]), ...
    'Bryson 输入权重倍数必须为 4x1。');
assert(all(isfinite(parameter.bryson_state_error_limit)) && all(parameter.bryson_state_error_limit > 0.0), ...
    'Bryson 状态误差边界必须为有限正数。');
assert(all(isfinite(parameter.bryson_state_weight_multiplier)) && ...
    all(parameter.bryson_state_weight_multiplier > 0.0), 'Bryson 状态权重倍数必须为有限正数。');
assert(all(isfinite(parameter.bryson_input_delta_limit)) && all(parameter.bryson_input_delta_limit > 0.0), ...
    'Bryson 输入增量边界必须为有限正数。');
assert(all(isfinite(parameter.bryson_input_weight_multiplier)) && ...
    all(parameter.bryson_input_weight_multiplier > 0.0), 'Bryson 输入权重倍数必须为有限正数。');

parameter.Q = diag(parameter.bryson_state_weight_multiplier ./ parameter.bryson_state_error_limit .^ 2);
parameter.R = diag(parameter.bryson_input_weight_multiplier ./ parameter.bryson_input_delta_limit .^ 2);
parameter.linearization_state_step = 1e-6;
parameter.linearization_input_step = 1e-6;
end
