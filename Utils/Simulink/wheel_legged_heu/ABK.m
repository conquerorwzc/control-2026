function Generate_LQR_Integrated()
% 主函数：集成计算、拟合与代码生成
clc; clear; close all;

fprintf('正在进行动力学建模与LQR计算，请稍候...\n');

%% ================= 第一部分：动力学建模与LQR计算 =================

% L0的变化范围
L0s = 0.112 : 0.01 : 0.38;

% 初始化存储空间
% 有length(L0s)个2x6的K矩阵
Ks = zeros(2, 6, length(L0s));

% 循环计算每个长度下的K值
% 注意：由于符号运算较慢，此循环可能需要几十秒到几分钟
for step = 1: length(L0s)
    if mod(step, 5) == 0
        fprintf('进度: %d / %d\n', step, length(L0s));
    end

    % 1.定义符号变量
    syms theta theta_d theta_dd;       % theta, theta_dot, theta_ddot
    syms x x_d x_dd;                   % x, x_dot, x_ddot
    syms phi phi_d phi_dd;             % phi, phi_dot, phi_ddot
    syms T Tp N P Nm Pm Nf t;

    % 2.机器人结构参数
    R = 0.077;
    L = L0s(step) / 2;
    Lm = L0s(step) / 2;
    l = 0;
    mw = 0.7;
    mp = 1.3;
    M = 5;
    Iw = 0.5 * mw * R^2;
    Ip = mp*((Lm+L)^2+0.05^2)/12.0;
    Im = M*(0.38^2+0.3^2)/12.0;
    g = 9.8;

    % 3.描述各个量之间的等量关系
    % 3.1 受力
    Nm = M*(x_dd + (L+Lm)*(cos(theta)*theta_dd - sin(theta)*theta_d^2) - l*(cos(phi)*phi_dd - sin(phi)*phi_d^2));
    Pm = M*g - M*((L+Lm)*(cos(theta)*theta_d^2 + sin(theta)*theta_dd) + l*(cos(phi)*phi_d^2 + sin(phi)*phi_dd));
    N = Nm + mp*(x_dd + L*(cos(theta)*theta_dd - sin(theta)*theta_d^2));
    P = Pm + mp*(g - L*(cos(theta)*theta_d^2 + sin(theta)*theta_dd));

    % 3.2 力矩
    equ1 = x_dd*(Iw/R+mw*R) + N*R - T;
    equ2 = Ip*theta_dd-Tp+T+(N*L+Nm*Lm)*cos(theta)-(P*L+Pm*Lm)*sin(theta);
    equ3 = Im*phi_dd-Pm*l*sin(phi)-Nm*l*cos(phi)-Tp;

    % 3.3 解方程
    [x_dd, theta_dd, phi_dd] = solve(equ1, equ2, equ3, x_dd, theta_dd, phi_dd);

    % 4. 雅可比矩阵 (线性化)
    % A矩阵: X_dot与X的关系
    JA = jacobian([theta_d; theta_dd; x_d; x_dd; phi_d; phi_dd], [theta theta_d x x_d phi phi_d]);
    % B矩阵: X_dot与u的关系
    JB = jacobian([theta_d; theta_dd; x_d; x_dd; phi_d; phi_dd], [T Tp]);

    % 在平衡位置 (全0) 进行数值代入
    A_val = double(subs(JA, [theta theta_d x x_d phi phi_d], [0 0 0 0 0 0]));
    B_val = double(subs(JB, [theta theta_d x x_d phi phi_d], [0 0 0 0 0 0]));

    % 5. 离散化处理
    [G, H] = c2d(A_val, B_val, 0.005); % 采样时间为0.005

    % 6. 求解Ks (LQR)
    % 使用您代码中启用的参数
    %6.求解Ks
    % 李识予参数
    % LQR_Q = diag([1000 100 500 100 5000 10]);
    % LQR_R = diag([1 0.25]);

    % LQR_Q = diag([1 1 500 100 5000 1]);
    % LQR_R = diag([5 1]);
    % 山海机甲参数？
    % LQR_Q = diag([10 10 2068 914 6394 10]);
    % LQR_R = diag([10 1]);

    % 飞坡
    LQR_Q = diag([5000 10 500 100 25000 10]);
    LQR_R = diag([10 1]);
    % 能用 不太抖
    % LQR_Q = diag([1 1 500 100 5000 1]);
    % LQR_R = diag([10 1]);
    Ks(:,:,step) = dlqr(G,H,LQR_Q,LQR_R);

    Ks(:,:,step) = dlqr(G, H, LQR_Q, LQR_R);
end

fprintf('计算完成，正在进行多项式拟合...\n');

%% ================= 第二部分：多项式拟合 =================

% 对K的每个元素进行关于L0的拟合
% 定义符号变量 L0 用于构建最终的符号矩阵
syms L0 real;
LQR_K_sym = sym('LQR_K', [2 6]);
LQR_K_sym(:,:) = 0; % 初始化

% 存储系数用于后续直接生成C代码，避免重复提取
% 维度: [行, 列, 系数(4个: 3次,2次,1次,常数)]
coefficients_data = zeros(2, 6, 4);

for x_idx = 1:2
    for y_idx = 1:6
        % 提取当前元素随L0s变化的数据
        data_points = reshape(Ks(x_idx, y_idx, :), 1, length(L0s));

        % 3次多项式拟合
        p = polyfit(L0s, data_points, 3);

        % 存储系数 (polyfit返回的是 [p1 p2 p3 p4] 对应 high to low order)
        coefficients_data(x_idx, y_idx, :) = p;

        % 构建符号表达式 (为了验证或后续可能的matlabFunction用途)
        LQR_K_sym(x_idx, y_idx) = p(1)*L0^3 + p(2)*L0^2 + p(3)*L0 + p(4);
    end
end

% 可选：验证一下 L0=0.2 时的值
fprintf('验证 L0=0.2 时的 LQR_K 矩阵:\n');
disp(double(subs(LQR_K_sym, L0, 0.2)));

%% ================= 第三部分：生成 C 代码并复制 =================

fprintf('正在生成 C 代码...\n');

[rows, cols] = size(LQR_K_sym);

% 打印C代码到命令行
fprintf('\n/* Generated LQR K Coefficients (Polynomial Degree 3) */\n');
fprintf('.LQR_K_Coefficient = {\n');
for i = 1:rows
    fprintf('    {');
    for j = 1:cols
        fprintf('{');
        for k = 1:4
            % coefficients_data 已经是 [3次, 2次, 1次, 常数] 顺序
            fprintf('%.15gf', coefficients_data(i, j, k));
            if k < 4
                fprintf(', ');
            end
        end
        fprintf('}');

        if j < cols
            fprintf(',      \\\n     ');
        end
    end
    if i < rows
        fprintf('},      \\\n');
    else
        fprintf('}},     \\\n');
    end
end
fprintf('\n');

% 生成剪贴板文本
clipboard_text = generate_clipboard_text_internal(coefficients_data, rows, cols);
clipboard('copy', clipboard_text);
fprintf('代码已复制到剪贴板！\n');

end

%% 辅助函数：生成剪贴板文本
function text = generate_clipboard_text_internal(coefficients, rows, cols)
text = sprintf('.LQR_K_Coefficient = {\\\n');

for i = 1:rows
    text = [text, sprintf('    {')]; %#ok<*AGROW>
    for j = 1:cols
        text = [text, sprintf('{')];
        for k = 1:4
            text = [text, sprintf('%.15gf', coefficients(i, j, k))];
            if k < 4
                text = [text, sprintf(', ')];
            end
        end
        text = [text, sprintf('}')];

        if j < cols
            text = [text, sprintf(',      \\\n     ')];
        end
    end

    if i < rows
        text = [text, sprintf('},      \\\n')];
    else
        % 保留反斜杠用于宏定义换行
        text = [text, sprintf('}},     \\')];
    end
end
end
