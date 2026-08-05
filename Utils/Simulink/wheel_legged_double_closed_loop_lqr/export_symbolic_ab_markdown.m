% export_symbolic_ab_markdown.m
% 导出双闭环模型的原始 Newton-Euler 方程、自动消元后的 M、B、g 与运动学约束。
%
% 注意: 本文件不导出控制器线性 A/B。控制器 A/B 由
% compute_lqr_and_export.m 在各 (x_ref, u0) 处对完整非线性状态方程求雅可比。

clear;
clc;

fprintf('========================================\n');
fprintf('导出双闭环符号动力学 Markdown\n');
fprintf('========================================\n\n');

%% ========================================
%  Step 1: 读取符号模型
%  ========================================

fprintf('Step 1: 读取符号模型...\n');
if ~isfile('double_closed_loop_symbolic_model.mat')
    build_symbolic_model;
end
model = load('double_closed_loop_symbolic_model.mat', 'raw_equations', 'generalized_equations', ...
    'kinematic_substitution_symbols', 'kinematic_substitution_values', 'M', 'B_control', 'g_vector');
file_id = fopen('double_closed_loop_symbolic_ab.md', 'w');
if file_id < 0
    error('Cannot open double_closed_loop_symbolic_ab.md for writing.');
end

%% ========================================
%  Step 2: 写入模型合同和假设
%  ========================================

fprintf('Step 2: 写入模型合同、假设和自动消元方程...\n');
fprintf(file_id, '# 双闭环等效腿模型：符号动力学 M、B、g 与约束\n\n');
fprintf(file_id, '## 状态和输入合同\n\n');
fprintf(file_id, '状态顺序：`[s, s_dot, phi, phi_dot, theta_L, theta_L_dot, theta_R, theta_R_dot, theta_b, theta_b_dot]`。\n\n');
fprintf(file_id, '输入顺序（模型正方向）：`[Tp_R, Tp_L, Tw_R, Tw_L]`。MCU 输入方向在数值 LQR 脚本中另行映射。\n\n');
fprintf(file_id, '$$\ns=s_w-s_{w,0}.\n$$\n\n');
fprintf(file_id, 's 是左右轮端平均滚动得到的纯轮式纵向二维坐标，不做 Yaw 的世界系投影。髋点 O 里程计只作为固件诊断量，不参与十维状态和 LQR。模型假设：平地、纯滚动、两轮接地、无 Roll、固定腿长工作点。左右地面对轮法向力相等用于闭合竖直内力；这是降阶模型前提，不是运行时接触力判断。\n\n');
fprintf(file_id, 'Tw 正值定义为使对应轮向前滚动的**轮端力矩**。轮对腿的反作用力矩因此为 `-Tw`；此处定义不等于未来 H6215 电机轴正指令，后者仍需独立标定。\n\n');
fprintf(file_id, '测量端髋点速度可包含腿长变化项；本符号动力学的单个工作点不把 `l_dot/l_ddot` 纳入状态。\n\n');
fprintf(file_id, '## 原始 Newton-Euler 方程\n\n');
fprintf(file_id, '以下 17 条残差包含机体、左右腿、左右轮、Yaw 方程和等法向力闭合。程序先消去 12 个接触内力，再得到五条广义坐标方程；不维护手写的 `eq1` 至 `eq5`。\n\n');
for index = 1:numel(model.raw_equations)
    write_equation(file_id, sprintf('r_{%d}', index), model.raw_equations(index));
end
fprintf(file_id, '## 自动消元后的五条广义方程\n\n');
for index = 1:numel(model.generalized_equations)
    write_equation(file_id, sprintf('E_{%d}', index), model.generalized_equations(index));
end

%% ========================================
%  Step 3: 写入运动学代换
%  ========================================

fprintf('Step 3: 写入运动学约束...\n');
fprintf(file_id, '## 纯轮式 s 运动学代换\n\n');
fprintf(file_id, '对每一侧：\n\n');
write_latex_block(file_id, 'R\theta_{w,L}=s-b\phi+\frac{q_R-q_L}{2},\qquad R\theta_{w,R}=s+b\phi+\frac{q_L-q_R}{2}.');
write_latex_block(file_id, 's=s_w=\frac{R\theta_{w,L}+R\theta_{w,R}}{2},\qquad q_i=l_i\sin\theta_i.');
fprintf(file_id, '以下为固定腿长工作点所用的加速度代换：\n\n');
for index = 1:numel(model.kinematic_substitution_symbols)
    fprintf(file_id, '$$\n%s=%s.\n$$\n\n', latex(model.kinematic_substitution_symbols(index)), ...
        latex(model.kinematic_substitution_values(index)));
end

%% ========================================
%  Step 4: 写入 M、B、g
%  ========================================

fprintf('Step 4: 写入 M、B、g 非零元素...\n');
fprintf(file_id, '## 广义方程\n\n');
write_latex_block(file_id, 'M(q)\ddot q=B_{\mathrm{control}}(q)u+g(q,\dot q).');
fprintf(file_id, '未列出的元素严格为零。\n\n');
write_nonzero_entries(file_id, 'M', model.M);
write_nonzero_entries(file_id, 'B_{\mathrm{control}}', model.B_control);
write_nonzero_entries(file_id, 'g', model.g_vector);

fprintf(file_id, '## 线性化边界\n\n');
fprintf(file_id, '控制器矩阵 `A`、`B` 不是在零输入处单独导出的符号式，而是 `compute_lqr_and_export.m` 在每个静态平衡点 `(x_ref,u0)` 对完整非线性状态方程求数值雅可比。因此保留了 `B(q)u0` 的姿态耦合。\n');
fclose(file_id);

fprintf('  输出: double_closed_loop_symbolic_ab.md\n');
fprintf('========================================\n');
fprintf('Markdown 导出完成\n');
fprintf('========================================\n');

%% ========================================
%  Private functions
%  ========================================

function write_equation(file_id, name, expression)
% 把一个符号表达式写成独立的 Markdown 块公式。
fprintf(file_id, '$$\n%s=%s.\n$$\n\n', name, latex(expression));
end

function write_latex_block(file_id, equation)
% 以 %s 参数写入手写 LaTeX，防止 fprintf 将反斜杠解释为格式转义。
fprintf(file_id, '$$\n%s\n$$\n\n', equation);
end

function write_nonzero_entries(file_id, symbol_name, matrix)
% 仅导出非零元素，既保持可审查性又避免整矩阵的零项噪声。
% 将可能已含下标的矩阵基名放入 \left(\right)，避免 B_{control}_{i,j} 双下标。
fprintf(file_id, '### $%s$ 的非零元素\n\n', symbol_name);
for row = 1:size(matrix, 1)
    for column = 1:size(matrix, 2)
        if ~isequal(matrix(row, column), sym(0))
            fprintf(file_id, '$$\n\\left(%s\\right)_{%d,%d}=%s.\n$$\n\n', symbol_name, row, column, ...
                latex(matrix(row, column)));
        end
    end
end
end
