% export_symbolic_ab_markdown.m
% 导出双闭环模型的符号动力学 M、B、g、五条原始方程与运动学约束。
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
model = load('double_closed_loop_symbolic_model.mat', 'eq1', 'eq2', 'eq3', 'eq4', 'eq5', ...
    'substitution_lhs', 'substitution_rhs', 'M', 'B_control', 'g_vector');
file_id = fopen('double_closed_loop_symbolic_ab.md', 'w');
if file_id < 0
    error('Cannot open double_closed_loop_symbolic_ab.md for writing.');
end

%% ========================================
%  Step 2: 写入模型合同和假设
%  ========================================

fprintf('Step 2: 写入模型合同、假设和五条方程...\n');
fprintf(file_id, '# 双闭环等效腿模型：符号动力学 M、B、g 与约束\n\n');
fprintf(file_id, '## 状态和输入合同\n\n');
fprintf(file_id, '状态顺序：`[s, s_dot, phi, phi_dot, theta_L, theta_L_dot, theta_R, theta_R_dot, theta_b, theta_b_dot]`。\n\n');
fprintf(file_id, '输入顺序（模型正方向）：`[Tp_R, Tp_L, Tw_R, Tw_L]`。MCU 输入方向在数值 LQR 脚本中另行映射。\n\n');
fprintf(file_id, '$$\ns=x_O-x_{O,0}.\n$$\n\n');
fprintf(file_id, '模型假设：平地、纯滚动、两轮接地、无 Roll、固定腿长工作点。`eq3/eq4` 额外采用左右地面对轮法向力相等的闭合假设；这是对称降阶模型的前提，不是运行时接触力判断。\n\n');
fprintf(file_id, '测量端髋点速度可包含腿长变化项；本符号动力学的单个工作点不把 `l_dot/l_ddot` 纳入状态。\n\n');
fprintf(file_id, '## 五条已化简动力学方程\n\n');
write_equation(file_id, 'eq_1', model.eq1);
write_equation(file_id, 'eq_2', model.eq2);
write_equation(file_id, 'eq_3', model.eq3);
write_equation(file_id, 'eq_4', model.eq4);
write_equation(file_id, 'eq_5', model.eq5);

%% ========================================
%  Step 3: 写入运动学代换
%  ========================================

fprintf('Step 3: 写入运动学约束...\n');
fprintf(file_id, '## 髋点 O 运动学代换\n\n');
fprintf(file_id, '对每一侧：\n\n');
write_latex_block(file_id, 's_{O,i}=R\theta_{w,i}+l_i\sin\theta_i,');
write_latex_block(file_id, '\dot{s}_{O,i}=R\dot{\theta}_{w,i}+\dot l_i\sin\theta_i+l_i\dot{\theta}_i\cos\theta_i.');
fprintf(file_id, '以下为固定腿长工作点所用的加速度代换：\n\n');
for index = 1:numel(model.substitution_lhs)
    fprintf(file_id, '$$\n%s=%s.\n$$\n\n', latex(model.substitution_lhs(index)), ...
        latex(model.substitution_rhs(index)));
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
