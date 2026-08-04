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
% s 是髋点 O 的世界系前向位移。左右腿分别由轮端滚动和虚拟腿
% 几何估计髋点位置，取平均后减去采样零点。该测量定义包含 l_dot，
% 但单个 LQR 工作点仍采用固定腿长近似，忽略 l_dot、l_ddot 的动力学影响。

%% ========================================
%  Step 1: 状态与输入合同
%  ========================================

parameter.state_order = {'s', 's_dot', 'phi', 'phi_dot', 'theta_L', ...
    'theta_L_dot', 'theta_R', 'theta_R_dot', 'theta_b', 'theta_b_dot'};
parameter.input_order = {'Tp_R', 'Tp_L', 'Tw_R', 'Tw_L'};

%% ========================================
%  Step 2: 重力与几何参数
%  ========================================

parameter.g = -9.81;
parameter.wheel_radius = 0.060;
parameter.half_track = 0.304 / 2;

%% ========================================
%  Step 3: 机身、虚拟腿与轮参数
%  ========================================

parameter.body_mass_without_battery = 1.535 + 0.108;
parameter.battery_mass = 0.0;
parameter.body_mass = parameter.body_mass_without_battery + parameter.battery_mass;
parameter.body_pitch_inertia = 0.0064;
parameter.body_com_to_pitch_axis = 0.0;
parameter.body_com_offset = 0.0;
parameter.yaw_inertia = 0.0845;

parameter.leg_mass_left = 0.206;
parameter.leg_mass_right = 0.206;
parameter.leg_inertia_left = 0.000962;
parameter.leg_inertia_right = 0.000962;
parameter.leg_com_offset_left = deg2rad(17.22);
parameter.leg_com_offset_right = deg2rad(17.22);

parameter.wheel_mass_left = 0.650;
parameter.wheel_mass_right = 0.650;
parameter.wheel_inertia_left = 0.000896;
parameter.wheel_inertia_right = 0.000896;

parameter.leg_com_distance = @(leg_length) 0.5 * leg_length;

%% ========================================
%  Step 4: 腿长调度与静态工作点
%  ========================================

parameter.leg_length_min = 0.140;
parameter.leg_length_max = 0.180;
parameter.leg_length_step = 0.005;
parameter.nominal_leg_length_left = 0.160;
parameter.nominal_leg_length_right = 0.160;

% 自动配平保持机身 Pitch 为此参考值，并求左右虚拟腿静态角和 u0。
parameter.trim_body_pitch_reference = 0.0;
parameter.trim_initial_leg_angle = 0.0;
parameter.trim_relative_residual_tolerance = 1e-7;
parameter.trim_angle_regularization = 1e-8;
parameter.trim_input_regularization = 1e-10;

%% ========================================
%  Step 5: 模型到 MCU 的输入方向
%  ========================================

% 模型中的正 Tp 使 theta_L/theta_R 减小；固件 VMC 中正 Tp 使虚拟腿角增大，
% 所以前两项由模型映射到 MCU 时固定取 -1。Tw 的真实执行器方向仍需真机验证。
parameter.input_sign = [-1; -1; 1; 1];
parameter.input_sign_status = {'Tp 与 VMC 合同已对齐', 'Tp 与 VMC 合同已对齐', ...
    'Tw_R 需真机验证', 'Tw_L 需真机验证'};

%% ========================================
%  Step 6: LQR 与数值线性化参数
%  ========================================

parameter.Q = diag([300, 30, 300, 10, 300, 15, 300, 15, 6000, 80]);
parameter.R = diag([2, 2, 8, 8]);
parameter.linearization_state_step = 1e-6;
parameter.linearization_input_step = 1e-6;
end
