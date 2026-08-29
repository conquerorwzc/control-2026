%% Generate_MPC_Coeffs.m
% 独立脚本：生成平衡步兵 LQR 串级 MPC 的拟合系数
% 并自动复制为 C 语言数组格式到剪贴板
clear; clc; close all;

%% 1. 参数定义
L0s = 0.12 : 0.01 : 0.37; % 腿长范围 (m)

% 机器人物理参数 (请根据实际 CAD 数据微调)
R = 0.077;          % 轮半径
mw = 0.7;           % 轮子质量
mp = 1.3;           % 摆杆质量 (约)
M = 14.0;           % 机体质量 (Robot Mass)
g = 9.8;

% 存储计算结果 [输入维度 x 状态维度 x 采样点数]
% 输入: [T(轮子), Tp(关节)]
% 状态: [theta, theta_d, x, x_d, phi, phi_d]
MPC_Ks_Data = zeros(2, 6, length(L0s));

% MPC 权重参数 (核心调参区)
% Q: 惩罚状态增量 (Delta x) -> 越大越抑制状态突变
% R: 惩罚控制增量 (Delta u) -> 越大控制越柔顺
Q_MPC = diag([10, 1, 10, 10, 10, 1]); 
R_MPC = diag([5, 5]); 

Ts = 0.002; % 控制周期 2ms (500Hz) 或 1ms (1kHz)，需与单片机一致

fprintf('正在计算 MPC 增益...\n');

%% 2. 符号动力学构建 (一次性构建，循环内代入数值)
syms theta theta_d theta_dd
syms x x_d x_dd
syms phi phi_d phi_dd
syms T Tp
syms L Lm Ip Im % 这些是随腿长变化的参数

% 动力学方程 (参考常见二阶倒立摆模型)
% 受力中间变量
Nm = M*(x_dd + (L+Lm)*(cos(theta)*theta_dd - sin(theta)*theta_d^2) - 0*(cos(phi)*phi_dd - sin(phi)*phi_d^2)); % l=0简化
Pm = M*g - M*((L+Lm)*(cos(theta)*theta_d^2 + sin(theta)*theta_dd) + 0*(cos(phi)*phi_d^2 + sin(phi)*phi_dd));
N = Nm + mp*(x_dd + L*(cos(theta)*theta_dd - sin(theta)*theta_d^2));
P = Pm + mp*(g - L*(cos(theta)*theta_d^2 + sin(theta)*theta_dd));

% 力矩方程
equ1 = x_dd*(0.5*mw*R^2/R + mw*R) + N*R - T == 0;
equ2 = Ip*theta_dd - Tp + T + (N*L + Nm*Lm)*cos(theta) - (P*L + Pm*Lm)*sin(theta) == 0;
equ3 = Im*phi_dd - Pm*0*sin(phi) - Nm*0*cos(phi) - Tp == 0; % l=0

% 求解加速度项
sol = solve([equ1, equ2, equ3], [x_dd, theta_dd, phi_dd]);

% 雅可比线性化 (在零点线性化)
JA = jacobian([sol.theta_dd; sol.x_dd; sol.phi_dd], [theta, theta_d, x, x_d, phi, phi_d]);
JB = jacobian([sol.theta_dd; sol.x_dd; sol.phi_dd], [T, Tp]);

JA_subs = subs(JA, [theta, theta_d, x, x_d, phi, phi_d], zeros(1,6));
JB_subs = subs(JB, [theta, theta_d, x, x_d, phi, phi_d], zeros(1,6));

%% 3. 循环计算增益
for i = 1:length(L0s)
    L_val = L0s(i) / 2.0;
    Lm_val = L0s(i) / 2.0;
    
    % 更新随 L0 变化的惯量
    % 注意：这里简化估算，实际应根据 CAD 拟合惯量函数
    Ip_val = mp * ((Lm_val + L_val)^2 + 0.05^2) / 12.0;
    Im_val = M * (0.3^2 + 0.3^2) / 12.0; 
    
    % 代入数值计算连续状态空间矩阵 Ac, Bc
    % 状态顺序: [theta, theta_d, x, x_d, phi, phi_d]
    % 注意 solve 出来的顺序可能不同，需重排
    % 假设 solve 顺序为 [theta_dd, x_dd, phi_dd]
    
    % 构造完整 A 矩阵 (6x6)
    % x_dot = [theta_d; theta_dd; x_d; x_dd; phi_d; phi_dd]
    A_sub = double(subs(JA_subs, {L, Lm, Ip, Im}, {L_val, Lm_val, Ip_val, Im_val}));
    B_sub = double(subs(JB_subs, {L, Lm, Ip, Im}, {L_val, Lm_val, Ip_val, Im_val}));
    
    Ac = zeros(6,6);
    Ac(1,2) = 1; Ac(3,4) = 1; Ac(5,6) = 1;
    % 填入动力学部分 (注意 JA_subs 的行顺序对应 sol 的输出顺序)
    % 这里需人工确认 sol 输出顺序，通常是按照 solve 参数顺序
    Ac(2,:) = A_sub(1,:); % theta_dd
    Ac(4,:) = A_sub(2,:); % x_dd
    Ac(6,:) = A_sub(3,:); % phi_dd
    
    Bc = zeros(6,2);
    Bc(2,:) = B_sub(1,:);
    Bc(4,:) = B_sub(2,:);
    Bc(6,:) = B_sub(3,:);
    
    % 离散化
    sys_d = c2d(ss(Ac, Bc, [], []), Ts);
    
    % 计算增量式 LQR (即 MPC 无约束解)
    K_val = dlqr(sys_d.A, sys_d.B, Q_MPC, R_MPC);
    
    MPC_Ks_Data(:,:,i) = K_val;
end

%% 4. 曲线拟合与代码生成 (修正版V4 - 高精度/无头无尾)
fprintf('正在拟合曲线并生成代码...\n');

% 拟合模型: a*exp(b*x) + c*exp(d*x)
ft = fittype('exp2'); 
opts = fitoptions(ft); 
opts.StartPoint = [1 0 1 0];
opts.Display = 'Off';

% 初始化输出字符串
output_str = '';

% 1. 移除了注释行
% 2. 字段定义头
output_str = [output_str, sprintf('      .MPC_K_Coefficient = { \\\n')];

for row = 1:2
    % 行缩进
    output_str = [output_str, sprintf('                            {')];
    
    for col = 1:6
        y_data = squeeze(MPC_Ks_Data(row, col, :));
        
        % 拟合
        [curve, ~] = fit(L0s', y_data, ft, opts);
        coeffs = coeffvalues(curve); % [a, b, c, d]
        
        % 格式化参数: %.12f 强制保留12位小数，末尾加f
        param_str = sprintf('{%.12ff, %.12ff, %.12ff, %.12ff}', ...
            coeffs(1), coeffs(2), coeffs(3), coeffs(4));
        
        output_str = [output_str, param_str];
        
        % 列分隔符
        if col < 6
            output_str = [output_str, ', '];
        end
    end
    
    % 行结束处理
    if row < 2
        % 第一行：闭合花括号，逗号，续行符，换行
        output_str = [output_str, sprintf('}, \\\n')];
    else
        % 最后一行：双重闭合，逗号(为下一个结构体成员准备)，续行符
        % 注意：这里特意去掉了 \n，光标将停在最后一行末尾
        output_str = [output_str, sprintf('}}, \\')];
    end
end

% 打印预览
fprintf('------------------------------------------------------\n');
fprintf('生成结果如下 (已复制到剪贴板):\n');
fprintf('------------------------------------------------------\n');
disp(output_str);

% 复制到剪贴板
clipboard('copy', output_str);
