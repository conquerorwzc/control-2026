# 双闭环等效腿 LQR 工具链

## 项目概述

本目录为双轮、左右独立等效虚拟腿的五广义坐标模型生成固定 0.170 m 腿长 LQR。建模脚本从原始 Newton-Euler 方程自动消去接触内力，输出符号模型、数值结果和 C 常量；本目录**不接入、不修改**底盘闭环控制链。

控制器和状态估计共用如下位置定义：

$$
s=s_w-s_{w,0},\qquad s_w=\frac{R_w\theta_{w,L}+R_w\theta_{w,R}}{2}.
$$

模型中的 s 是左右轮端平均滚动得到的纯轮式纵向坐标，s_dot 是对应速度。髋点 O 里程计仍保留在固件中作为诊断对比量，但不参与十维状态和 LQR。

当前公式不对 Yaw phi 作 cos/sin 投影；s 不是固定世界 X 坐标，也不能用于大 Yaw 机动中的全局定位。若要维护全局 X/Y，必须在状态估计层增加完整平面坐标变换；本次没有扩展模型维度。

### 原文勘误和采用边界

模型以 `WBR_modeling.html` 的 (3.1) 至 (3.10) 原始 Newton--Euler 方程为基准，但不原样复制已确认的错误：

- 原文 (3.14) 的 `-I_b*ddtheta_l,r` 是笔误，必须为 `-I_b*ddtheta_b`，否则机身惯量会错误作用到右腿角加速度。
- 原文对 `l_b,i` 的文字释义有误；结合 (2.4)、(3.5)，它是机体--腿关节到腿质心的距离，并满足 `l_w,i+l_b,i=l_i`。
- 原文 (2.2)、(2.4) 未完整加入非零机身质心偏置的几何投影，属于降阶近似。符号模型保留原文 (3.8) 的 `l_c` 髋点力矩项，但当前参数仍强制 `body_com_to_hip_distance=0`；若要改成非零，必须先补全机身运动学闭环，不能只改参数后沿用当前 K。
- 原文把等效腿质心放在腿轴线上，没有独立质心偏角。工具链不再暴露未参与方程的 `leg_com_offset` 参数；若 CAD 表明质心明显偏离腿轴，必须同时重建腿质心运动学和转动方程，不能只添加一个角度常量。
- 原文使用 `s/h` 表示水平/竖直方向；源码内部也采用 `_s/_h` 命名，避免把原文的竖直下标 `h` 与英文 `horizontal` 混淆。

上述三点与 `WBR_modeling_每个式子推导详解.md` 的勘误结论一致。其余本轮使用的轮、腿、机身和 Yaw 力矩正方向均按原文 (3.2)、(3.5)、(3.8)、(3.9) 实现，并由符号偏导断言检查作用--反作用。

## 文件结构

```text
wheel_legged_double_closed_loop_lqr/
├── double_closed_loop_parameters.m     # 参数、状态/输入合同、方向映射、LQR 配置
├── build_symbolic_model.m              # 原文 15 条方程自动消元 -> M、B、g
├── export_symbolic_ab_markdown.m       # 导出原始方程、约束、M、B、g Markdown
├── compute_lqr_and_export.m            # 静态配平、全非线性线性化、LQR、C 导出
├── verify_double_closed_loop_lqr.m     # 符号、约束、Tw 合同、配平和导出交叉检查
├── double_closed_loop_symbolic_model.mat       # build 生成，禁止手改
├── double_closed_loop_symbolic_ab.md           # export 生成，禁止手改
├── double_closed_loop_lqr_results.mat          # compute 生成，禁止手改
├── double_closed_loop_lqr_response.md          # compute 生成的十状态/四输入响应合同
└── double_closed_loop_lqr_coefficients.h       # compute 生成，禁止手改
```

## 运行顺序

1. 在 MATLAB 中进入本目录，确认 `double_closed_loop_parameters.m` 的质量、惯量、几何和 Q/R。
2. 运行 `build_symbolic_model`，生成 M(q)、B_control(q)、g(q,dq)。
3. 运行 `export_symbolic_ab_markdown`，人工审查原文 15 条方程、自动消元结果和运动学代换。
4. 运行 `compute_lqr_and_export`，在固定 0.170 m 工作点求静态平衡，再由完整非线性状态方程数值线性化，导出 K 和全状态第一拍响应合同。
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
| 0, 1 | s, s_dot | 左右轮端平均滚动纵向坐标、速度 | m, m/s |
| 2, 3 | phi, phi_dot | 世界系 Yaw 和角速度；车头向左转为正 | rad, rad/s |
| 4, 5 | theta_L, theta_L_dot | 左腿含机身 Pitch 的纵向平面腿角和角速度 | rad, rad/s |
| 6, 7 | theta_R, theta_R_dot | 右腿含机身 Pitch 的纵向平面腿角和角速度 | rad, rad/s |
| 8, 9 | theta_b, theta_b_dot | 机身 Pitch 和角速度；机身前端向下为正 | rad, rad/s |

### 正负号合同

采用右手世界系：x 向初始车头前方，y 向左，z 向上。除非特别说明，角速度的正方向与相应角度增大的方向一致。

| 量 | 零位 | 正方向 | 负方向 |
| --- | --- | --- | --- |
| s | 初始化时锁存的左右轮平均滚动坐标 s_w0 | 左右轮端平均向前滚动，s 增大 | 左右轮端平均向后滚动，s 减小 |
| phi | 车头与初始世界 x 轴对齐 | 绕世界 +z 轴右手旋转，车头向左转 | 车头向右转 |
| theta_L | 左轮轴 P_L 正下方对应左髋点 O，左虚拟腿竖直 | 从 P_L 指向 O 的向量向 +x 倾斜，即 O 相对 P_L 向前、P_L 相对 O 向后 | O 相对 P_L 向后、P_L 相对 O 向前 |
| theta_R | 右轮轴 P_R 正下方对应右髋点 O，右虚拟腿竖直 | 从 P_R 指向 O 的向量向 +x 倾斜，即 O 相对 P_R 向前、P_R 相对 O 向后 | O 相对 P_R 向后、P_R 相对 O 向前 |
| theta_b | 机身纵向轴与世界 +x 平行 | 绕世界 +y 轴右手旋转，机身前端向下 | 机身前端向上 |

左右腿使用**同一套**纵向平面角度定义，不能因为一条腿在左侧、另一条腿在右侧而人为给 theta_L、theta_R 加相反符号。腿角几何关系为：

$$
x_O-x_{P,i}=l_i\sin\theta_i,
$$

$$
z_O-z_{P,i}=l_i\cos\theta_i.
$$

因此 theta_i 等于零时虚拟腿竖直；theta_i 增大时髋点 O 相对轮轴 P 向前。s 是轮端平均滚动坐标，不是髋点 O 的实际纵向坐标，也不做 Yaw 投影，不能直接当作全局 X 位移。

原文模型输入顺序固定为：

$$
u_{model}=[T_{lw,L},\ T_{lw,R},\ T_{bl,L},\ T_{bl,R}]^T.
$$

MCU 使用语义更短的同号别名，但行顺序不同：

$$
u_{\mathrm{mcu}}=[Tp_R,\ Tp_L,\ Tw_R,\ Tw_L]^T.
$$

两者通过固定置换关系连接：

$$
u_{model}=
\begin{bmatrix}
0&0&0&1\\
0&0&1&0\\
0&1&0&0\\
1&0&0&0
\end{bmatrix}u_{mcu}.
$$

Tp 是虚拟腿空间的广义力矩，必须经闭链 Jacobian 映射成关节力矩；Tw 是使对应轮向前滚动的**轮端力矩**。两者都不能因为变量同名而直接等同于电机电流或单个关节力矩。

与 SJTU 原文 (3.2)、(3.5)、(3.8) 的符号映射固定为：

$$
Tw_L=T_{lw,L},\qquad Tw_R=T_{lw,R},
$$

$$
Tp_L=T_{bl,L},\qquad Tp_R=T_{bl,R}.
$$

其中 T_lw 是腿上电机施加给轮的驱动力矩，T_bl 是机身施加给腿的髋关节力矩；当前 Tp 与 T_bl 使用同一定义。因此，同一执行器在相邻刚体上的力矩必须成对出现：

| 执行器 | 轮 | 腿 | 机身 |
| --- | --- | --- | --- |
| Tw_i | `+Tw_i` | `-Tw_i` | 0 |
| Tp_i | 0 | `+Tp_i` | `-Tp_i` |

代入原文腿方程后，髋关节和轮电机项为 `-Tw_i+Tp_i`；机身方程中的髋关节项为 `-Tp_L-Tp_R`。`input_sign` 只负责模型与 MCU 坐标映射，不能用于修补违反作用反作用的原始方程；当前模型与 VMC 的 Tp 定义相同，所以 Tp 不取反。

| 输入 | 模型正方向 | MCU 正方向 | 确认状态 |
| --- | --- | --- | --- |
| Tp_R | 机身施加给右腿的髋关节力矩；右腿受 `+Tp_R`、机身受 `-Tp_R`，使 theta_R 增大、theta_b 减小 | `input_sign=+1`，正 MCU Tp_R 与模型 Tp_R 同方向 | 软件合同已对齐 |
| Tp_L | 机身施加给左腿的髋关节力矩；左腿受 `+Tp_L`、机身受 `-Tp_L`，使 theta_L 增大、theta_b 减小 | `input_sign=+1`，正 MCU Tp_L 与模型 Tp_L 同方向 | 软件合同已对齐 |
| Tw_R | 正值使右轮向前滚动，且在固定直立点使 phi_ddot 增大 | `input_sign=+1`，不额外取反 | 轮端定义已固定；H6215 输出方向待标定 |
| Tw_L | 正值使左轮向前滚动，且在固定直立点使 phi_ddot 减小 | `input_sign=+1`，不额外取反 | 轮端定义已固定；H6215 输出方向待标定 |

Tw_R、Tw_L 的表述是模型轮端广义力通道的正负号，不是 H6215 电机轴正指令的已验证结论。当前 `input_sign=+1` 已使 MCU LQR 输出与上述轮端定义一致；真实闭环前仍必须用低幅单轮 Tw、手推和 Yaw 单轴试验标定 H6215 方向。

本车是 1:1 直驱，输出层不再乘 `wheel.direction`。左右安装镜像只能通过 H6215 的 `motor_reverse_flag`、`feedback_reverse_flag` 成对配置，使“正软件命令”和“正软件反馈”同时对应轮端前进；`wheel.direction` 固定为 `+1`。若手推前进的反馈符号不对，禁止只改 `wheel.direction`，否则会造成状态坐标改变而电机输出坐标不变。

当前参数固定 `body_com_to_hip_distance=0`，即机身质心位于髋点 O。符号模型已经保留 SJTU 原文 (3.8) 的髋点水平力和竖直力矩，但原文 (2.2)/(2.4) 的运动学在非零 `l_c` 下不严格闭合；因此若以后使用非零机身质心偏距，仍须先补全运动学，再重新消元、线性化和生成 K，不能只修改长度参数。

## 纯轮式 s 与左右约束

纯轮式 s 的左右约束为：

$$
R_w\theta_{w,L}=s-b\phi+\frac{q_R-q_L}{2},
$$

$$
R_w\theta_{w,R}=s+b\phi+\frac{q_L-q_R}{2},
$$

其中 q_i=l_i\sin\theta_i。两式相加可得纯轮式状态：

$$
s_w=\frac{R_w\theta_{w,L}+R_w\theta_{w,R}}{2}.
$$

因此轮子不动时腿角或腿长变化不会改变 s/s_dot。固件仍计算下面的髋点量用于诊断，但不会写入十维状态：

$$
s_{O,L}=r\theta_{w,L}+l_L\sin\theta_{L,\mathrm{world}},
$$

$$
s_{O,R}=r\theta_{w,R}+l_R\sin\theta_{R,\mathrm{world}},
$$

$$
s_{O}=\frac{s_{O,L}+s_{O,R}}{2}.
$$

髋点诊断速度仍保留三个物理项：

$$
\dot{s}_{O,L}=r\dot\theta_{w,L}+\dot l_L\sin\theta_{L,\mathrm{world}}+l_L\dot\theta_{L,\mathrm{world}}\cos\theta_{L,\mathrm{world}},
$$

$$
\dot{s}_{O,R}=r\dot\theta_{w,R}+\dot l_R\sin\theta_{R,\mathrm{world}}+l_R\dot\theta_{R,\mathrm{world}}\cos\theta_{R,\mathrm{world}},
$$

$$
\dot{s}_{O}=\frac{\dot{s}_{O,L}+\dot{s}_{O,R}}{2}.
$$

s_w 的零点在左右轮、IMU 和左右腿首次完整有效采样时锁存；状态估计重置时必须连同 s_w,0 重置。轮编码器的轮向、轮径和无滑假设仍需真机确认；轮滑和离地风险尚未由 Kalman 处理。

纯轮式 s 下，动力学中的机身水平坐标不能再直接写成 s。由左右轮位置和腿的水平投影可得：

$$
x_b=s+\frac{q_L+q_R}{2},\qquad
a_{b,x}=\ddot{s}+\frac{\ddot q_L+\ddot q_R}{2}.
$$

如果只把轮角约束改成纯轮式形式、却仍使用 a_b,x=ddot{s}，就会把轮端坐标和机身坐标混用，导致生成的 A/B/K 不对应当前状态定义。

## 模型与假设

`build_symbolic_model.m` 以 SJTU 原文 (3.1) 至 (3.10) 的 15 条方程为起点：机体 3 条、左右腿各 3 条、左右轮各 2 条、Yaw 1 条和等支持力 1 条。程序先消去原文的 10 个接触内力，再以左右独立的纯轮式约束消去轮角加速度，整理为：

$$
M(q)\ddot q=B_{\mathrm{control}}(q)u+g(q,\dot q).
$$

模型适用于平地、纯滚动、两轮接地、无 Roll 的降阶情形。原文 (3.10) 约束的是 `F_wh,L=F_wh,R`，即左右轮对腿的竖直支持力相等，不是左右地面法向力相等；它也不是运行时接触力测量或接触状态判断。左右轮必须分别使用 q_L、q_R 的二阶导数；不得用平均腿加速度代替，否则会丢失 Yaw 耦合。腿质心竖直加速度严格由原文 (2.4) 求导，不以左右各自刚性髋高替代原文的平均机身高度近似。

接触力的正方向单独定义：Tw 是轮电机施加给轮子的力矩；F_g_to_w 是地面对轮子的接触力；F_w_to_l 是轮子对腿的轴承/轮轴作用力。三者都以车体前方为正，但不能互相替代。零加速度的受力定义检查中，正 Tw 对应正的 F_g_to_w 和 F_w_to_l；真实动态响应还包含轮子平动惯性项，因此 F_w_to_l 的数值可以变负。这是同一正方向下的物理反作用，不是符号定义错误。

早期分析曾自定义 `T_wl_to_l`、`T_wr_to_r` 表示轮电机定子对腿的内部反作用扭矩；它们不是 SJTU 原文的控制输入。原文控制输入 `T_lw,i` 与本项目电机 Tw 同义，表示电机施加给轮的驱动力矩。当前符号模型另外显式定义其对腿的反作用：

$$
T_{\mathrm{wheel\to leg}}=-T_w.
$$

轮转动方程使用电机 Tw，腿转动方程使用相反的定子反作用扭矩；内部反作用不是另一组控制输入，也不能接到 H6215。

诊断用髋点速度采用完整的 l_dot 项；LQR 使用的纯轮式 s_dot 只来自左右轮速。一个 LQR 工作点的动力学仍固定 l_L、l_R，不把 l_dot、l_ddot 引入状态。这是刻意的模型边界，不能把“诊断公式包含 l_dot”误解成“模型已包含伸腿动力学”。

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

### Bryson 法 Q/R 调参

`double_closed_loop_parameters.m` 不再直接填写 Q、R。它按下式自动生成：

$$
Q_{ii}=\frac{q_i}{e_{i,\max}^2},
$$

$$
R_{jj}=\frac{r_j}{\Delta u_{j,\max}^2}.
$$

参数文件中的 `bryson_state_tuning` 是 10x2 状态调参表，每行依次为 `[e_max, q]`；`bryson_input_tuning` 是 4x2 输入调参表，每行依次为 `[delta_u_max, r]`。行顺序分别固定对应状态和输入顺序。脚本从两张表导出兼容字段 `bryson_state_error_limit`、`bryson_state_weight_multiplier`、`bryson_input_delta_limit` 和 `bryson_input_weight_multiplier`。

第一列 e_max、delta_u_max 是有物理单位的成本归一化分母，必须为有限正数：e_max 表示可接受状态误差，delta_u_max 表示相对静态配平输入 u0 希望使用的控制增量。它们不是机械限位、软件限幅、电机额定值或实际输出保护。第二列 q、r 是无量纲正倍数：q 增大表示更重视压制对应状态误差；r 增大表示更克制使用对应输入。日常调参优先修改第二列；数学上 q 乘 4 与 e_max 除以 2 会产生相同的 Q 对角元素，但前者不改变物理边界的语义。改变表中任一数值后，必须重跑 `compute_lqr_and_export` 和 `verify_double_closed_loop_lqr`，再更新 MCU LQR 的导出常量。

当前初值按控制优先级设置为：机身俯仰最高、左右腿角较高、髋点位移中等、偏航较低，所有速度项先取基础阻尼权重 1。它们是首次仿真的设计起点，不是已完成真机验证的最终参数；未来实际输出仍必须经过 VMC/轮毂独立的限幅、变化率限制和故障回退。

A、B 在 `(x_ref,u0)` 对完整非线性状态方程作中心差分雅可比，而不是只对 g 求导，因此包含 B(q)u0 的姿态耦合。当前第一版固定在 `l_L=l_R=0.170 m` 的单一工作点，只导出该点的 K、x_ref 和 u0，不做腿长调度或多项式拟合。MCU 在接入 Tp/Tw 前自动执行腿长准备，只有双腿稳定在 0.170 m 附近且腿角、Pitch 处于捕获范围时才进入平衡阶段；这不是 K 调度，仍不能把固定 K 外推到大腿长偏差。

## MATLAB 到 MCU 接口

参数中固定：

```matlab
parameter.input_sign = [1; 1; 1; 1];
```

数值化时：

$$
B_{\mathrm{mcu}}=B_{\mathrm{control}}\operatorname{diag}(\mathrm{input\_sign}).
$$

Tp_R、Tp_L 与 SJTU 的 T_bl 和当前 VMC Pitch 广义力矩同向：正 Tp 是机身对腿的力矩，并使对应虚拟腿角朝正方向变化。Tw_R、Tw_L 与 SJTU 的 T_lw 同向，固定为轮端前进驱动力矩；H6215 的电机轴正指令仍需真机验证。一旦更改任何输入符号，必须重新生成 K、x_ref、u0。

注意：标准无约束 LQR 是四输入耦合优化，不能要求“单独 s>0 时每个 Tw 初始都必须为负”。当前标称模型的 `B_{dot s,Tw_R}`、`B_{dot s,Tw_L}` 均为正，说明正 Tw 确实使 s 加速度增大；但四输入最优分配得到的 `K(Tw,s)` 为负，故 `-Kx` 的第一拍 Tw 可能为正。它不是 `B_control` 或 `input_sign` 反号的证据，不能在 MCU 输出端手动取负；若要求单状态误差第一拍必然反向，必须另行设计带输入结构约束的外环或输入分配器，并重新验证闭环。

这也解释了手扶试验中容易出现的“轮子继续跑”：纯轮式位置控制具有非最小相位回正过程，控制器可能先让轮子沿误差方向短暂运动，以建立腿角和机身 Pitch 后再反向收回。若测试者把机身和腿强行保持竖直，这条状态响应被外力阻断，控制器会持续尝试建立它，不能据此判断 Tw 应整体取反。位置闭环只能在保护架允许小角度自然响应时验收；刚性手扶只适合检查单路命令方向、限幅和急停。

当前公共 Tw 到纯轮式 s 的线性化通道存在右半平面不变零点。验证脚本会自动检查该事实，因此 LQR 验收应以闭环极点、有限时间状态响应和实际饱和行为为准，而不是以某一个输入在第一拍的符号为准。

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
