# 双闭环等效腿 LQR 工具链

## 项目概述

本目录为双轮、左右独立等效虚拟腿的五广义坐标模型生成固定腿长 LQR，并对左右腿长进行二维二次增益调度。建模脚本保持单文件流程，输出符号模型、数值结果和 C 常量；本目录**不接入、不修改**底盘闭环控制链。

控制器和状态估计共用如下位置定义：

$$
s=x_O-x_{O,0}.
$$

O 是机身髋点，s 是 O 在世界系前向水平方向的位移，s_dot 是其世界系前向速度。

当前公式没有对 Yaw phi 作 cos/sin 投影，因此“世界系前向”在本降阶模型中默认初始世界前向与车体前向对齐，并适用于小 Yaw 的纵向平面。若要在大 Yaw 机动中维护全局 X/Y 位置，必须在状态估计层增加完整平面坐标变换；本次没有扩展模型维度。

## 文件结构

```text
wheel_legged_double_closed_loop_lqr/
├── double_closed_loop_parameters.m     # 参数、状态/输入合同、方向映射、LQR 配置
├── build_symbolic_model.m              # 五条已化简动力学方程 -> M、B、g
├── export_symbolic_ab_markdown.m       # 导出五条方程、约束、M、B、g Markdown
├── compute_lqr_and_export.m            # 静态配平、全非线性线性化、LQR、二维拟合、C 导出
├── verify_double_closed_loop_lqr.m     # 符号、里程计、配平、线性化和接口交叉检查
├── double_closed_loop_symbolic_model.mat       # build 生成，禁止手改
├── double_closed_loop_symbolic_ab.md           # export 生成，禁止手改
├── double_closed_loop_lqr_results.mat          # compute 生成，禁止手改
└── double_closed_loop_lqr_coefficients.h       # compute 生成，禁止手改
```

## 运行顺序

1. 在 MATLAB 中进入本目录，确认 `double_closed_loop_parameters.m` 的质量、惯量、几何和 Q/R。
2. 运行 `build_symbolic_model`，生成 M(q)、B_control(q)、g(q,dq)。
3. 运行 `export_symbolic_ab_markdown`，人工审查五条方程和运动学代换。
4. 运行 `compute_lqr_and_export`，每个左右腿长点先求静态平衡，再由完整非线性状态方程数值线性化并导出。
5. 运行 `verify_double_closed_loop_lqr`；未通过时不得使用导出的 K。

## 状态和输入定义

广义坐标为：

$$
q=[s,\ \phi,\ \theta_L,\ \theta_R,\ \theta_b]^T.
$$

状态顺序固定为：

$$
x=[s,\ \dot s,\ \phi,\ \dot\phi,\ \theta_L,\ \dot\theta_L,\ \theta_R,\ \dot\theta_R,\ \theta_b,\ \dot\theta_b]^T.
$$

| 索引 | 状态 | 含义 | 单位 |
| --- | --- | --- | --- |
| 0, 1 | s, s_dot | 髋点 O 世界系前向位移、速度 | m, m/s |
| 2, 3 | phi, phi_dot | 世界系 Yaw 和角速度；车头向左转为正 | rad, rad/s |
| 4, 5 | theta_L, theta_L_dot | 左腿世界系虚拟腿角和角速度 | rad, rad/s |
| 6, 7 | theta_R, theta_R_dot | 右腿世界系虚拟腿角和角速度 | rad, rad/s |
| 8, 9 | theta_b, theta_b_dot | 机身 Pitch 和角速度；机身前端向下为正 | rad, rad/s |

### 正负号合同

采用右手世界系：x 向初始车头前方，y 向左，z 向上。除非特别说明，角速度的正方向与相应角度增大的方向一致。

| 量 | 零位 | 正方向 | 负方向 |
| --- | --- | --- | --- |
| s | 初始化时锁存的髋点 O 位置 s0 | 髋点 O 向世界系前向移动，s 增大 | 髋点 O 向后移动，s 减小 |
| phi | 车头与初始世界 x 轴对齐 | 绕世界 +z 轴右手旋转，车头向左转 | 车头向右转 |
| theta_L | 左轮轴 P_L 正下方对应左髋点 O，左虚拟腿竖直 | 从 P_L 指向 O 的向量向 +x 倾斜，即 O 相对 P_L 向前、P_L 相对 O 向后 | O 相对 P_L 向后、P_L 相对 O 向前 |
| theta_R | 右轮轴 P_R 正下方对应右髋点 O，右虚拟腿竖直 | 从 P_R 指向 O 的向量向 +x 倾斜，即 O 相对 P_R 向前、P_R 相对 O 向后 | O 相对 P_R 向后、P_R 相对 O 向前 |
| theta_b | 机身纵向轴与世界 +x 平行 | 绕世界 +y 轴右手旋转，机身前端向下 | 机身前端向上 |

左右腿使用**同一套**世界系角度定义，不能因为一条腿在左侧、另一条腿在右侧而人为给 theta_L、theta_R 加相反符号。腿角几何关系为：

$$
x_O-x_{P,i}=l_i\sin\theta_i,
$$

$$
z_O-z_{P,i}=l_i\cos\theta_i.
$$

因此 theta_i 等于零时虚拟腿竖直；theta_i 增大时 O 的前向坐标增加。s 的“世界系前向”定义在当前模型中以初始车头方向为参考；当 Yaw 很大时，必须将车体前向位移投影到世界系，不能直接把该一维 s 当作全局 X 位移。

输入顺序固定为：

$$
u_{\mathrm{mcu}}=[Tp_R,\ Tp_L,\ Tw_R,\ Tw_L]^T.
$$

Tp 是虚拟腿空间的广义力矩，必须经闭链 Jacobian 映射成关节力矩；Tw 是轮对虚拟腿的反作用力矩。两者都不能因为变量同名而直接等同于电机电流或单个关节力矩。

| 输入 | 模型正方向 | MCU 正方向 | 确认状态 |
| --- | --- | --- | --- |
| Tp_R | 使模型 theta_R 减小、theta_b 增大的广义力矩方向 | `input_sign=-1` 后，正 MCU Tp_R 对应模型负 Tp_R，即使 theta_R 增大的 VMC 合同方向 | 软件合同已对齐 |
| Tp_L | 使模型 theta_L 减小、theta_b 增大的广义力矩方向 | `input_sign=-1` 后，正 MCU Tp_L 对应模型负 Tp_L，即使 theta_L 增大的 VMC 合同方向 | 软件合同已对齐 |
| Tw_R | 在模型 Yaw 广义方程中进入负通道：B_control(5,3)=-R_w/r | 当前 `input_sign=+1`，未翻转模型符号 | 需真机验证 |
| Tw_L | 在模型 Yaw 广义方程中进入正通道：B_control(5,4)=+R_w/r | 当前 `input_sign=+1`，未翻转模型符号 | 需真机验证 |

Tw_R、Tw_L 的表述是**模型广义力通道**的正负号，不是轮电机、电机编码器或轮子实际前进方向的已验证结论。轮端和传动方向必须通过悬空低幅 Tw、手推和 Yaw 单轴试验确认。

## 髋点 O 里程计

左右腿独立估计髋点，再取平均。定义为：

$$
s_{O,L}=r\theta_{w,L}+l_L\sin\theta_{L,\mathrm{world}},
$$

$$
s_{O,R}=r\theta_{w,R}+l_R\sin\theta_{R,\mathrm{world}},
$$

$$
s=\frac{s_{O,L}+s_{O,R}}{2}-s_0.
$$

每侧速度必须保留三个物理项：

$$
\dot{s}_{O,L}=r\dot\theta_{w,L}+\dot l_L\sin\theta_{L,\mathrm{world}}+l_L\dot\theta_{L,\mathrm{world}}\cos\theta_{L,\mathrm{world}},
$$

$$
\dot{s}_{O,R}=r\dot\theta_{w,R}+\dot l_R\sin\theta_{R,\mathrm{world}}+l_R\dot\theta_{R,\mathrm{world}}\cos\theta_{R,\mathrm{world}},
$$

$$
\dot{s}=\frac{\dot{s}_{O,L}+\dot{s}_{O,R}}{2}.
$$

s0 在里程计首次完整有效采样时锁存；状态估计重置时必须连同 s0 重置。固件现有实现已经采用上述公式，但轮编码器、IMU 和世界系腿角的物理正方向仍需真机确认。

## 模型与假设

`build_symbolic_model.m` 以五条已化简的 Newton-Euler 方程为起点，并用固定腿长工作点的髋点运动学约束消去轮加速度，整理为：

$$
M(q)\ddot q=B_{\mathrm{control}}(q)u+g(q,\dot q).
$$

模型适用于平地、纯滚动、两轮接地、无 Roll 的降阶情形。`eq3` 与 `eq4` 明确增加：左右地面对轮法向力相等。这是左右对称支持下的**模型闭合假设**，不是运行时接触力测量或接触状态判断。

测量端 s_dot 采用完整的 l_dot 项；但一个 LQR 工作点的动力学仍固定 l_L、l_R，不把 l_dot、l_ddot 引入状态。这是刻意的模型边界，不能把“测量公式完整”误解成“模型已包含伸腿动力学”。

## 静态平衡与 LQR

每个左右腿长样本固定 theta_b 参考值，优化左右腿角并求 u0，使：

$$
B(q_{\mathrm{ref}})u_0+g(q_{\mathrm{ref}},0)=0.
$$

脚本输出归一化广义力残差；残差高于 `trim_relative_residual_tolerance` 时立即报错，绝不导出伪平衡 K。

控制器为：

$$
u_{\mathrm{mcu}}=u_0-K(x-x_{\mathrm{ref}}).
$$

A、B 在 `(x_ref,u0)` 对完整非线性状态方程作中心差分雅可比，而不是只对 g 求导，因此包含 B(q)u0 的姿态耦合。K、x_ref、u0 均使用：

$$
[1,\ l_L,\ l_R,\ l_L^2,\ l_Ll_R,\ l_R^2]
$$

作二维二次拟合；C 头文件同时保留标称值和调度系数。腿长超出参数文件给出的拟合范围时，调用端必须限幅或禁用调度，不能外推。

## MATLAB 到 MCU 接口

参数中固定：

```matlab
parameter.input_sign = [-1; -1; 1; 1];
```

数值化时：

$$
B_{\mathrm{mcu}}=B_{\mathrm{control}}\operatorname{diag}(\mathrm{input\_sign}).
$$

Tp_R、Tp_L 的负号已经按当前 VMC 软件合同对齐：模型正 Tp 减小虚拟腿角，VMC 正 Pitch 力矩增大虚拟腿角。Tw_R、Tw_L 的符号仍标记为“需真机验证”；一旦更改任何输入符号，必须重新生成 K、x_ref、u0 和所有拟合系数。

导出的 `double_closed_loop_lqr_coefficients.h` 仅是常量数据，不包含限幅、状态有效性、失效回退、闭链 Jacobian 映射或电机输出逻辑，不能被直接当作完整闭环控制器。

## 真机验收

MATLAB 符号恒等、残差、可控性和闭环极点只证明当前数学模型与脚本一致，不证明 CAD 参数、传感器方向、轮地接触和执行器方向已正确。

真机按低限幅、单自由度、可靠保护的顺序验证：

1. 静态零点：检查左右世界系腿角与机身 Pitch 的零位。
2. 单轴 Pitch：缓慢使机身前端向下、向上，检查 theta_b 与 theta_b_dot 符号。
3. 手推和纯滚动：检查 s 与 s_dot 的轮滚动、腿摆和腿长变化项方向。
4. 单轮：分别施加小 Tw_R、Tw_L，确认前进与 Yaw 方向。
5. 单腿 Tp：经 Jacobian 映射后检查正负 Tp 对虚拟腿角的影响。
6. 闭环前：确认状态/输入顺序、左右索引、力矩限幅、状态有效条件与失效回退。

在上述试验有记录前，Tw、轮编码器、IMU、传动方向和接触假设均为“需真机验证”。
