% 测试验证
L0 = 0.170;

% 调用您的函数
LQR_K_result = lqr_K(L0);

fprintf('Matlab计算结果 (L0 = %.3f):\n', L0);
disp(LQR_K_result);

fprintf('\n详细输出:\n');
for i = 1:2
    for j = 1:6
        fprintf('LQR_K(%d,%d) = %.10f\n', i, j, LQR_K_result(i,j));
    end
end
