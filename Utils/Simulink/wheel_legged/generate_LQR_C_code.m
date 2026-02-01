function generate_LQR_C_code()
% 专门用于生成LQR_K系数的C代码
% 直接使用您的lqr_K函数
    % 创建符号变量
    syms L0 real
    
    % 调用您的函数
    LQR_K_sym = lqr_K(L0);
    
    % 矩阵尺寸
    [rows, cols] = size(LQR_K_sym);
    
    % 提取系数
    coefficients = zeros(rows, cols, 4);
    
    for i = 1:rows
        for j = 1:cols
            % 展开表达式
            poly_expr = expand(LQR_K_sym(i, j));
            
            % 提取系数
            c = sym2poly(poly_expr);
            
            % 补齐到4个系数（如果低于3次）
            if length(c) < 4
                c = [zeros(1, 4-length(c)), c];
            end
            
            % 存储 [L0^3, L0^2, L0^1, 常数]
            coefficients(i, j, :) = c;
        end
    end
    
    % 打印C代码到命令行 (为了视觉效果，这里保留换行)
    fprintf('.LQR_K_Coefficient = {\n');
    for i = 1:rows
        fprintf('    {');
        for j = 1:cols
            fprintf('{');
            for k = 1:4
                fprintf('%.15gf', coefficients(i, j, k));
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
    
    % 保存到剪贴板 (已修复末尾多余空行的问题)
    clipboard_text = generate_clipboard_text(coefficients, rows, cols);
    clipboard('copy', clipboard_text);
    fprintf('代码已复制到剪贴板！\n');
end

function text = generate_clipboard_text(coefficients, rows, cols)
    text = sprintf('.LQR_K_Coefficient = {\\\n');
    
    for i = 1:rows
        text = [text, sprintf('    {')];
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
            % 【修改处】: 这里去掉了最后的 \n，只保留了反斜杠
            text = [text, sprintf('}},     \\')];
        end
    end
end