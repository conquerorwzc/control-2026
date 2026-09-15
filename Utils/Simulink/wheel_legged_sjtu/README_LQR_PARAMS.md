# LQR 参数生成与填入 C 端说明

本目录用于生成 SJTU 轮腿底盘 4×10 LQR 的 `K[4][10]` 与腿长二维拟合系数 `LQR_K_Coefficients[40][6]`，并填入嵌入式配置。

## 步骤

1. **生成线性化系统**（若尚未有 `linearized_system.mat`）
  在 MATLAB 中于**本目录**执行：
   得到 `linearized_system.mat`。
2. **计算 LQR 并打印 C 代码**
  在本目录执行：
   脚本会：
  - 计算标称腿长下的 4×10 增益矩阵 K；
  - 在腿长网格上采样并做二维多项式拟合，得到 40×6 系数；
  - 在**终端**输出可直接复制的 C 代码块。
3. **填入 robot_config.h**
  打开：
   找到 `lqr_param` 的 `.K = {{0}}` 与 `.LQR_K_Coefficients = {{0}}`：
  - 将 `compute_lqr.m` 输出的 `float K[4][10] = { ... };` 内容整理成 `.K = { ... }` 填入；
  - 将 `.LQR_K_Coefficients = { ... }` 整块复制到配置中对应位置。
4. **（可选）验证**
  - 用 `lqr_results.mat` 在 MATLAB 中检查闭环特征值、可控性；
  - 若后续增加 `test_lqr.m`，可做闭环仿真与 C 端数值一致性检查。

## 注意

- 未填入有效 K 与系数时，`robot_config.h` 中为全零占位，底盘**无法平衡与速度跟踪**（审计报告 P0）。
- 状态与符号约定见 `chassis_wheel_legged_sjtu/chassis.c` 中 `StateVarUpdate()` 的注释，须与推导文档/线性化一致。

