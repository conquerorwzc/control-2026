%% ============================================================
%  推导 T4(phi4) 和 T1(phi1)，以 C 语言格式输出
%  ★ 修改下方 k_val 即可重新生成对应的 C 代码 ★
%% ============================================================
clear; clc;

%% ========== 可调参数 ==========
k_val = 800;          %  ← 在此修改弹簧刚度 (N/m)，改完直接重新运行

%% ========== 符号变量 ==========
syms phi4 phi1

%% ========== 已知常量 ==========
L1     = 0.06403;                         % m
L2     = 0.12135;                         % m
L0     = 0.120;                           % m
theta1 = 9.01  * pi / 180;               % rad
theta2 = 38.66 * pi / 180;               % rad
theta3 = 9.05  * pi / 180;               % rad
k      = k_val;                           % N/m

%% ============================================================
%  推导链 —— phi4
%% ============================================================
alpha4   = pi - (phi4 - theta1) - theta2;
x4       = sqrt(L1^2 + L2^2 - 2*L1*L2*cos(alpha4));
F4       = (x4 - L0) * k;
sinBeta4 = L1 * sin(alpha4 + theta3) / x4;
T4       = simplify(sinBeta4 * F4 * L2);

%% ============================================================
%  推导链 —— phi1
%% ============================================================
alpha1   = phi1 - theta2 + theta1;
x1       = sqrt(L1^2 + L2^2 - 2*L1*L2*cos(alpha1));
F1       = (x1 - L0) * k;
sinBeta1 = L1 * sin(alpha1 + theta3) / x1;
T1       = simplify(sinBeta1 * F1 * L2);

%% ============================================================
%  输出 C 代码
%% ============================================================
fprintf('\n/*============================================*/\n');
fprintf('/*  自动生成   k = %.4f N/m                  */\n', k_val);
fprintf('/*============================================*/\n\n');

fprintf('#include <math.h>\n\n');

% ---------- T4(phi4) ----------
fprintf('/* T4 = f(phi4)  [N·m] */\n');
fprintf('double T4(double phi4)\n{\n');
fprintf('    double t;\n');
ccode_T4 = ccode(T4);                         % 自动生成 C 表达式
% ccode 返回 "  t0 = ...;\n" 的形式，替换变量名
ccode_T4 = strrep(ccode_T4, 't0', 't');
fprintf('%s\n', ccode_T4);
fprintf('    return t;\n}\n\n');

% ---------- T1(phi1) ----------
fprintf('/* T1 = f(phi1)  [N·m] */\n');
fprintf('double T1(double phi1)\n{\n');
fprintf('    double t;\n');
ccode_T1 = ccode(T1);
ccode_T1 = strrep(ccode_T1, 't0', 't');
fprintf('%s\n', ccode_T1);
fprintf('    return t;\n}\n\n');

% ---------- 附赠：带 k 参数的通用版 ----------
fprintf('/*============================================*/\n');
fprintf('/*  通用版：k 作为函数参数                     */\n');
fprintf('/*============================================*/\n\n');

syms k_sym
T4_gen = simplify(subs(T4, k, 1) * k_sym);    % 因为 T 与 k 成正比
T1_gen = simplify(subs(T1, k, 1) * k_sym);    % 提出 k 即可

% --- T4 通用 ---
fprintf('double T4_general(double phi4, double k)\n{\n');
fprintf('    double t;\n');
c4g = ccode(T4_gen);
c4g = strrep(c4g, 't0', 't');
c4g = strrep(c4g, 'k_sym', 'k');
fprintf('%s\n', c4g);
fprintf('    return t;\n}\n\n');

% --- T1 通用 ---
fprintf('double T1_general(double phi1, double k)\n{\n');
fprintf('    double t;\n');
c1g = ccode(T1_gen);
c1g = strrep(c1g, 't0', 't');
c1g = strrep(c1g, 'k_sym', 'k');
fprintf('%s\n', c1g);
fprintf('    return t;\n}\n');

fprintf('\n/* ---- 生成完毕 ---- */\n');