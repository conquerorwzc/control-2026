# 双闭环等效腿模型：符号动力学 M、B、g 与约束

## 状态和输入合同

状态顺序：`[s, s_dot, phi, phi_dot, theta_L, theta_L_dot, theta_R, theta_R_dot, theta_b, theta_b_dot]`。

原文模型输入顺序：`[T_lw,L, T_lw,R, T_bl,L, T_bl,R]`。其中 `T_bl` 是机身对腿的力矩，`T_lw` 是腿/电机对轮的驱动力矩。MCU 顺序为 `[Tp_R, Tp_L, Tw_R, Tw_L]`，由数值脚本显式置换。

$$
s=s_w-s_{w,0}.
$$

s 是左右轮端平均滚动得到的纯轮式纵向二维坐标，不做 Yaw 的世界系投影。髋点 O 里程计只作为固件诊断量，不参与十维状态和 LQR。模型假设：平地、纯滚动、两轮接地、无 Roll、固定腿长工作点。按原文 (3.10) 使用 F_wh,L=F_wh,R，即左右轮对腿的竖直支持力相等；这是降阶模型前提，不是运行时接触力判断。

`T_bl,i` 对腿为正、对机身为负；`T_lw,i` 对轮为正、对腿为负。正 `T_lw` 定义为使对应轮向前滚动的轮端力矩；此处定义不等于未来 H6215 电机轴正指令，后者仍需独立标定。

测量端髋点速度可包含腿长变化项；本符号动力学的单个工作点不把 `l_dot/l_ddot` 纳入状态。

## 原始 Newton-Euler 方程

以下 15 条残差严格对应原文 (3.1) 至 (3.10)：机体 3 条、左右腿各 3 条、左右轮各 2 条、Yaw 1 条和等支持力 1 条。程序先消去 10 个接触内力，再得到五条广义坐标方程；不维护手写的 `eq1` 至 `eq5`。

$$
r_{1}=F_{l,\mathrm{to},b,s}+F_{r,\mathrm{to},b,s}-a_{b,s}\,m_{b}.
$$

$$
r_{2}=F_{l,\mathrm{to},b,h}+F_{r,\mathrm{to},b,h}-a_{b,h}\,m_{b}-g\,m_{b}.
$$

$$
r_{3}=l_{c}\,\sin\left(\theta _{b}\right)\,\left(F_{l,\mathrm{to},b,h}+F_{r,\mathrm{to},b,h}\right)-T_{\mathrm{bl},r}-I_{b}\,\mathrm{ddtheta}_{b}-l_{c}\,\cos\left(\theta _{b}\right)\,\left(F_{l,\mathrm{to},b,s}+F_{r,\mathrm{to},b,s}\right)-T_{\mathrm{bl},l}.
$$

$$
r_{4}=F_{\mathrm{wl},\mathrm{to},l,s}-F_{l,\mathrm{to},b,s}-a_{l,s}\,m_{l}.
$$

$$
r_{5}=F_{\mathrm{wl},\mathrm{to},l,h}-F_{l,\mathrm{to},b,h}-a_{l,h}\,m_{l}-g\,m_{l}.
$$

$$
r_{6}=T_{\mathrm{bl},l}-T_{\mathrm{lw},l}-I_{l}\,\mathrm{ddtheta}_{l}-\cos\left(\theta _{l}\right)\,\left(F_{\mathrm{wl},\mathrm{to},l,s}\,l_{l,d}+F_{l,\mathrm{to},b,s}\,\left(l_{l}-l_{l,d}\right)\right)+\sin\left(\theta _{l}\right)\,\left(F_{\mathrm{wl},\mathrm{to},l,h}\,l_{l,d}+F_{l,\mathrm{to},b,h}\,\left(l_{l}-l_{l,d}\right)\right).
$$

$$
r_{7}=F_{\mathrm{wr},\mathrm{to},r,s}-F_{r,\mathrm{to},b,s}-a_{r,s}\,m_{r}.
$$

$$
r_{8}=F_{\mathrm{wr},\mathrm{to},r,h}-F_{r,\mathrm{to},b,h}-a_{r,h}\,m_{r}-g\,m_{r}.
$$

$$
r_{9}=T_{\mathrm{bl},r}-T_{\mathrm{lw},r}-I_{r}\,\mathrm{ddtheta}_{r}-\cos\left(\theta _{r}\right)\,\left(F_{\mathrm{wr},\mathrm{to},r,s}\,l_{r,d}+F_{r,\mathrm{to},b,s}\,\left(l_{r}-l_{r,d}\right)\right)+\sin\left(\theta _{r}\right)\,\left(F_{\mathrm{wr},\mathrm{to},r,h}\,l_{r,d}+F_{r,\mathrm{to},b,h}\,\left(l_{r}-l_{r,d}\right)\right).
$$

$$
r_{10}=F_{g,\mathrm{to},\mathrm{wl},s}-F_{\mathrm{wl},\mathrm{to},l,s}-a_{\mathrm{wl},s}\,m_{\mathrm{wl}}.
$$

$$
r_{11}=T_{\mathrm{lw},l}-I_{\mathrm{wl}}\,\mathrm{ddtheta}_{\mathrm{wl}}-F_{g,\mathrm{to},\mathrm{wl},s}\,\mathrm{wheel}_{\mathrm{radius}}.
$$

$$
r_{12}=F_{g,\mathrm{to},\mathrm{wr},s}-F_{\mathrm{wr},\mathrm{to},r,s}-a_{\mathrm{wr},s}\,m_{\mathrm{wr}}.
$$

$$
r_{13}=T_{\mathrm{lw},r}-I_{\mathrm{wr}}\,\mathrm{ddtheta}_{\mathrm{wr}}-F_{g,\mathrm{to},\mathrm{wr},s}\,\mathrm{wheel}_{\mathrm{radius}}.
$$

$$
r_{14}=-I_{\mathrm{yaw}}\,\mathrm{ddphi}-\mathrm{half}_{\mathrm{track}}\,\left(F_{g,\mathrm{to},\mathrm{wl},s}-F_{g,\mathrm{to},\mathrm{wr},s}\right).
$$

$$
r_{15}=F_{\mathrm{wl},\mathrm{to},l,h}-F_{\mathrm{wr},\mathrm{to},r,h}.
$$

## 自动消元后的五条广义方程

$$
E_{1}=-\frac{2\,I_{\mathrm{wl}}\,\mathrm{dds}+2\,I_{\mathrm{wr}}\,\mathrm{dds}-2\,T_{\mathrm{lw},l}\,\mathrm{wheel}_{\mathrm{radius}}-2\,T_{\mathrm{lw},r}\,\mathrm{wheel}_{\mathrm{radius}}+2\,\mathrm{dds}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,\mathrm{dds}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,\mathrm{dds}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,\mathrm{dds}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,\mathrm{dds}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-2\,I_{\mathrm{wl}}\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}+2\,I_{\mathrm{wr}}\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}-I_{\mathrm{wl}}\,\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)+I_{\mathrm{wr}}\,\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)+I_{\mathrm{wl}}\,\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)-I_{\mathrm{wr}}\,\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)-2\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-2\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+I_{\mathrm{wl}}\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,\sin\left(\theta _{l}\right)-I_{\mathrm{wr}}\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,\sin\left(\theta _{l}\right)-I_{\mathrm{wl}}\,{\mathrm{dtheta}_{r}}^2\,l_{r}\,\sin\left(\theta _{r}\right)+I_{\mathrm{wr}}\,{\mathrm{dtheta}_{r}}^2\,l_{r}\,\sin\left(\theta _{r}\right)+\mathrm{ddtheta}_{l}\,l_{l}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)+\mathrm{ddtheta}_{r}\,l_{r}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)-\mathrm{ddtheta}_{l}\,l_{l}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)+2\,\mathrm{ddtheta}_{l}\,l_{l,d}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)+\mathrm{ddtheta}_{l}\,l_{l}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)+\mathrm{ddtheta}_{r}\,l_{r}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)-\mathrm{ddtheta}_{l}\,l_{l}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)+\mathrm{ddtheta}_{l}\,l_{l}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)-\mathrm{ddtheta}_{r}\,l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)+2\,\mathrm{ddtheta}_{r}\,l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)+\mathrm{ddtheta}_{r}\,l_{r}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)-\mathrm{ddtheta}_{r}\,l_{r}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)-{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{l}\right)-{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{r}\right)+{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{l}\right)-2\,{\mathrm{dtheta}_{l}}^2\,l_{l,d}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{l}\right)-{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{l}\right)-{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{r}\right)+{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{l}\right)-{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{l}\right)+{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{r}\right)-2\,{\mathrm{dtheta}_{r}}^2\,l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{r}\right)-{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{r}\right)+{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{r}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
E_{2}=-T_{\mathrm{bl},l}-T_{\mathrm{bl},r}-I_{b}\,\mathrm{ddtheta}_{b}-l_{c}\,\cos\left(\theta _{b}\right)\,\left(\frac{2\,T_{\mathrm{lw},l}\,\mathrm{wheel}_{\mathrm{radius}}-2\,I_{\mathrm{wl}}\,\mathrm{dds}-2\,\mathrm{dds}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-2\,\mathrm{dds}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,I_{\mathrm{wl}}\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}+I_{\mathrm{wl}}\,\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)-I_{\mathrm{wl}}\,\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)+2\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-I_{\mathrm{wl}}\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,\sin\left(\theta _{l}\right)+I_{\mathrm{wl}}\,{\mathrm{dtheta}_{r}}^2\,l_{r}\,\sin\left(\theta _{r}\right)+\mathrm{ddtheta}_{l}\,l_{l}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)-2\,\mathrm{ddtheta}_{l}\,l_{l,d}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)-\mathrm{ddtheta}_{r}\,l_{r}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)+\mathrm{ddtheta}_{l}\,l_{l}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)-\mathrm{ddtheta}_{r}\,l_{r}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)-{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{l}\right)+2\,{\mathrm{dtheta}_{l}}^2\,l_{l,d}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{l}\right)+{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{r}\right)-{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{l}\right)+{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{r}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{2\,I_{\mathrm{wr}}\,\mathrm{dds}-2\,T_{\mathrm{lw},r}\,\mathrm{wheel}_{\mathrm{radius}}+2\,\mathrm{dds}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,\mathrm{dds}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,I_{\mathrm{wr}}\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}+I_{\mathrm{wr}}\,\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)-I_{\mathrm{wr}}\,\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)+2\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-I_{\mathrm{wr}}\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,\sin\left(\theta _{l}\right)+I_{\mathrm{wr}}\,{\mathrm{dtheta}_{r}}^2\,l_{r}\,\sin\left(\theta _{r}\right)+\mathrm{ddtheta}_{l}\,l_{l}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)+\mathrm{ddtheta}_{l}\,l_{l}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)-\mathrm{ddtheta}_{r}\,l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)+2\,\mathrm{ddtheta}_{r}\,l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)-\mathrm{ddtheta}_{r}\,l_{r}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)-{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{l}\right)-{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{l}\right)+{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{r}\right)-2\,{\mathrm{dtheta}_{r}}^2\,l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{r}\right)+{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{r}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}\right)-\frac{l_{c}\,m_{b}\,\sin\left(\theta _{b}\right)\,\left(l_{l}\,\cos\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2+l_{r}\,\cos\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2-2\,g+\mathrm{ddtheta}_{l}\,l_{l}\,\sin\left(\theta _{l}\right)+\mathrm{ddtheta}_{r}\,l_{r}\,\sin\left(\theta _{r}\right)\right)}{2}.
$$

$$
E_{3}=T_{\mathrm{bl},l}-T_{\mathrm{lw},l}-I_{l}\,\mathrm{ddtheta}_{l}-\frac{\mathrm{ddtheta}_{l}\,{l_{l}}^2\,m_{l}\,{\cos\left(\theta _{l}\right)}^2}{2}-\mathrm{ddtheta}_{l}\,{l_{l,d}}^2\,m_{l}\,{\cos\left(\theta _{l}\right)}^2-\frac{\mathrm{ddtheta}_{l}\,{l_{l}}^2\,m_{\mathrm{wl}}\,{\cos\left(\theta _{l}\right)}^2}{2}-\frac{\mathrm{ddtheta}_{l}\,{l_{l}}^2\,m_{b}\,{\sin\left(\theta _{l}\right)}^2}{4}-\frac{\mathrm{ddtheta}_{l}\,{l_{l}}^2\,m_{l}\,{\sin\left(\theta _{l}\right)}^2}{4}-\mathrm{ddtheta}_{l}\,{l_{l,d}}^2\,m_{l}\,{\sin\left(\theta _{l}\right)}^2-\frac{\mathrm{ddtheta}_{l}\,{l_{l}}^2\,m_{r}\,{\sin\left(\theta _{l}\right)}^2}{4}+\mathrm{dds}\,l_{l}\,m_{l}\,\cos\left(\theta _{l}\right)-\mathrm{dds}\,l_{l,d}\,m_{l}\,\cos\left(\theta _{l}\right)+\mathrm{dds}\,l_{l}\,m_{\mathrm{wl}}\,\cos\left(\theta _{l}\right)+\frac{g\,l_{l}\,m_{b}\,\sin\left(\theta _{l}\right)}{2}-\frac{g\,l_{l}\,m_{l}\,\sin\left(\theta _{l}\right)}{2}+g\,l_{l,d}\,m_{l}\,\sin\left(\theta _{l}\right)+\frac{g\,l_{l}\,m_{r}\,\sin\left(\theta _{l}\right)}{2}-\frac{{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{b}\,\sin\left(2\,\theta _{l}\right)}{8}+\frac{{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{l}\,\sin\left(2\,\theta _{l}\right)}{8}-\frac{{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{r}\,\sin\left(2\,\theta _{l}\right)}{8}+\frac{{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{\mathrm{wl}}\,\sin\left(2\,\theta _{l}\right)}{4}-\frac{T_{\mathrm{lw},l}\,l_{l}\,\cos\left(\theta _{l}\right)}{\mathrm{wheel}_{\mathrm{radius}}}+\frac{I_{\mathrm{wl}}\,\mathrm{dds}\,l_{l}\,\cos\left(\theta _{l}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}+\frac{3\,\mathrm{ddtheta}_{l}\,l_{l}\,l_{l,d}\,m_{l}\,{\cos\left(\theta _{l}\right)}^2}{2}+\mathrm{ddtheta}_{l}\,l_{l}\,l_{l,d}\,m_{l}\,{\sin\left(\theta _{l}\right)}^2+\frac{I_{\mathrm{wl}}\,{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,\sin\left(2\,\theta _{l}\right)}{4\,{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{l,d}\,m_{l}\,\sin\left(2\,\theta _{l}\right)}{4}-\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,l_{l}\,m_{l}\,\cos\left(\theta _{l}\right)+\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,l_{l,d}\,m_{l}\,\cos\left(\theta _{l}\right)-\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,l_{l}\,m_{\mathrm{wl}}\,\cos\left(\theta _{l}\right)-\frac{I_{\mathrm{wl}}\,\mathrm{ddtheta}_{l}\,{l_{l}}^2\,{\cos\left(\theta _{l}\right)}^2}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,m_{b}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)}{4}-\frac{{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,m_{l}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2}+\frac{{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,m_{l}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)}{4}+\frac{{\mathrm{dtheta}_{r}}^2\,l_{l,d}\,l_{r}\,m_{l}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2}-\frac{{\mathrm{dtheta}_{r}}^2\,l_{l,d}\,l_{r}\,m_{l}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)}{2}+\frac{{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,m_{r}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)}{4}-\frac{{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r,d}\,m_{r}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)}{2}-\frac{{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,m_{\mathrm{wl}}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2}-\frac{I_{\mathrm{wl}}\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,l_{l}\,\cos\left(\theta _{l}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}+\frac{\mathrm{ddtheta}_{r}\,l_{l}\,l_{r}\,m_{l}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2}-\frac{\mathrm{ddtheta}_{r}\,l_{l,d}\,l_{r}\,m_{l}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2}+\frac{\mathrm{ddtheta}_{r}\,l_{l}\,l_{r}\,m_{\mathrm{wl}}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2}-\frac{\mathrm{ddtheta}_{r}\,l_{l}\,l_{r}\,m_{b}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}+\frac{\mathrm{ddtheta}_{r}\,l_{l}\,l_{r}\,m_{l}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}-\frac{\mathrm{ddtheta}_{r}\,l_{l,d}\,l_{r}\,m_{l}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2}+\frac{\mathrm{ddtheta}_{r}\,l_{l}\,l_{r}\,m_{r}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}-\frac{\mathrm{ddtheta}_{r}\,l_{l}\,l_{r,d}\,m_{r}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2}+\frac{I_{\mathrm{wl}}\,\mathrm{ddtheta}_{r}\,l_{l}\,l_{r}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{I_{\mathrm{wl}}\,{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
E_{4}=T_{\mathrm{bl},r}-T_{\mathrm{lw},r}-I_{r}\,\mathrm{ddtheta}_{r}-\frac{\mathrm{ddtheta}_{r}\,{l_{r}}^2\,m_{r}\,{\cos\left(\theta _{r}\right)}^2}{2}-\mathrm{ddtheta}_{r}\,{l_{r,d}}^2\,m_{r}\,{\cos\left(\theta _{r}\right)}^2-\frac{\mathrm{ddtheta}_{r}\,{l_{r}}^2\,m_{\mathrm{wr}}\,{\cos\left(\theta _{r}\right)}^2}{2}-\frac{\mathrm{ddtheta}_{r}\,{l_{r}}^2\,m_{b}\,{\sin\left(\theta _{r}\right)}^2}{4}-\frac{\mathrm{ddtheta}_{r}\,{l_{r}}^2\,m_{l}\,{\sin\left(\theta _{r}\right)}^2}{4}-\frac{\mathrm{ddtheta}_{r}\,{l_{r}}^2\,m_{r}\,{\sin\left(\theta _{r}\right)}^2}{4}-\mathrm{ddtheta}_{r}\,{l_{r,d}}^2\,m_{r}\,{\sin\left(\theta _{r}\right)}^2+\mathrm{dds}\,l_{r}\,m_{r}\,\cos\left(\theta _{r}\right)-\mathrm{dds}\,l_{r,d}\,m_{r}\,\cos\left(\theta _{r}\right)+\mathrm{dds}\,l_{r}\,m_{\mathrm{wr}}\,\cos\left(\theta _{r}\right)+\frac{g\,l_{r}\,m_{b}\,\sin\left(\theta _{r}\right)}{2}+\frac{g\,l_{r}\,m_{l}\,\sin\left(\theta _{r}\right)}{2}-\frac{g\,l_{r}\,m_{r}\,\sin\left(\theta _{r}\right)}{2}+g\,l_{r,d}\,m_{r}\,\sin\left(\theta _{r}\right)-\frac{{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{b}\,\sin\left(2\,\theta _{r}\right)}{8}-\frac{{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{l}\,\sin\left(2\,\theta _{r}\right)}{8}+\frac{{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{r}\,\sin\left(2\,\theta _{r}\right)}{8}+\frac{{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{\mathrm{wr}}\,\sin\left(2\,\theta _{r}\right)}{4}-\frac{T_{\mathrm{lw},r}\,l_{r}\,\cos\left(\theta _{r}\right)}{\mathrm{wheel}_{\mathrm{radius}}}+\frac{I_{\mathrm{wr}}\,\mathrm{dds}\,l_{r}\,\cos\left(\theta _{r}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}+\frac{3\,\mathrm{ddtheta}_{r}\,l_{r}\,l_{r,d}\,m_{r}\,{\cos\left(\theta _{r}\right)}^2}{2}+\mathrm{ddtheta}_{r}\,l_{r}\,l_{r,d}\,m_{r}\,{\sin\left(\theta _{r}\right)}^2+\frac{I_{\mathrm{wr}}\,{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,\sin\left(2\,\theta _{r}\right)}{4\,{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{{\mathrm{dtheta}_{r}}^2\,l_{r}\,l_{r,d}\,m_{r}\,\sin\left(2\,\theta _{r}\right)}{4}+\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,l_{r}\,m_{r}\,\cos\left(\theta _{r}\right)-\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,l_{r,d}\,m_{r}\,\cos\left(\theta _{r}\right)+\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,l_{r}\,m_{\mathrm{wr}}\,\cos\left(\theta _{r}\right)-\frac{I_{\mathrm{wr}}\,\mathrm{ddtheta}_{r}\,{l_{r}}^2\,{\cos\left(\theta _{r}\right)}^2}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,m_{b}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}+\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,m_{l}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l,d}\,l_{r}\,m_{l}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2}+\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,m_{r}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,m_{r}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)}{2}-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r,d}\,m_{r}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2}+\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r,d}\,m_{r}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)}{2}-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,m_{\mathrm{wr}}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)}{2}+\frac{I_{\mathrm{wr}}\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,l_{r}\,\cos\left(\theta _{r}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}+\frac{\mathrm{ddtheta}_{l}\,l_{l}\,l_{r}\,m_{r}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2}-\frac{\mathrm{ddtheta}_{l}\,l_{l}\,l_{r,d}\,m_{r}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2}+\frac{\mathrm{ddtheta}_{l}\,l_{l}\,l_{r}\,m_{\mathrm{wr}}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2}-\frac{\mathrm{ddtheta}_{l}\,l_{l}\,l_{r}\,m_{b}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}+\frac{\mathrm{ddtheta}_{l}\,l_{l}\,l_{r}\,m_{l}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}-\frac{\mathrm{ddtheta}_{l}\,l_{l,d}\,l_{r}\,m_{l}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2}+\frac{\mathrm{ddtheta}_{l}\,l_{l}\,l_{r}\,m_{r}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}-\frac{\mathrm{ddtheta}_{l}\,l_{l}\,l_{r,d}\,m_{r}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2}+\frac{I_{\mathrm{wr}}\,\mathrm{ddtheta}_{l}\,l_{l}\,l_{r}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{I_{\mathrm{wr}}\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
E_{5}=-I_{\mathrm{yaw}}\,\mathrm{ddphi}-\mathrm{half}_{\mathrm{track}}\,\left(\frac{-I_{\mathrm{wl}}\,l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2+I_{\mathrm{wl}}\,l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2-2\,I_{\mathrm{wl}}\,\mathrm{dds}+2\,T_{\mathrm{lw},l}\,\mathrm{wheel}_{\mathrm{radius}}+2\,I_{\mathrm{wl}}\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}+I_{\mathrm{wl}}\,\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)-I_{\mathrm{wl}}\,\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}+\frac{-I_{\mathrm{wr}}\,l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2+I_{\mathrm{wr}}\,l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2+2\,I_{\mathrm{wr}}\,\mathrm{dds}-2\,T_{\mathrm{lw},r}\,\mathrm{wheel}_{\mathrm{radius}}+2\,I_{\mathrm{wr}}\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}+I_{\mathrm{wr}}\,\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)-I_{\mathrm{wr}}\,\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}\right).
$$

## 纯轮式 s 运动学代换

对每一侧：

$$
R_w\theta_{w,L}=s-R_l\phi+\frac{q_R-q_L}{2},\qquad R_w\theta_{w,R}=s+R_l\phi+\frac{q_L-q_R}{2}.
$$

$$
s=s_w=\frac{R_w\theta_{w,L}+R_w\theta_{w,R}}{2},\qquad q_i=l_i\sin\theta_i.
$$

以下为固定腿长工作点所用的加速度代换：

$$
\mathrm{ddtheta}_{\mathrm{wl}}=\frac{\frac{l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2}{2}-\frac{l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2}{2}+\mathrm{dds}-\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}-\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)}{2}+\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{2}}{\mathrm{wheel}_{\mathrm{radius}}}.
$$

$$
\mathrm{ddtheta}_{\mathrm{wr}}=\frac{-\frac{l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2}{2}+\frac{l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2}{2}+\mathrm{dds}+\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}+\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)}{2}-\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{2}}{\mathrm{wheel}_{\mathrm{radius}}}.
$$

$$
a_{\mathrm{wl},s}=\frac{l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2}{2}-\frac{l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2}{2}+\mathrm{dds}-\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}-\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)}{2}+\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{2}.
$$

$$
a_{\mathrm{wr},s}=-\frac{l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2}{2}+\frac{l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2}{2}+\mathrm{dds}+\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}+\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)}{2}-\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{2}.
$$

$$
a_{l,s}=\mathrm{dds}-\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}-\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)}{2}+\mathrm{ddtheta}_{l}\,l_{l,d}\,\cos\left(\theta _{l}\right)+\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{2}+\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,\sin\left(\theta _{l}\right)}{2}-{\mathrm{dtheta}_{l}}^2\,l_{l,d}\,\sin\left(\theta _{l}\right)-\frac{{\mathrm{dtheta}_{r}}^2\,l_{r}\,\sin\left(\theta _{r}\right)}{2}.
$$

$$
a_{l,h}=\mathrm{ddtheta}_{l}\,\sin\left(\theta _{l}\right)\,\left(l_{l}-l_{l,d}\right)-\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\sin\left(\theta _{r}\right)}{2}-\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\sin\left(\theta _{l}\right)}{2}-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,\cos\left(\theta _{l}\right)}{2}-\frac{{\mathrm{dtheta}_{r}}^2\,l_{r}\,\cos\left(\theta _{r}\right)}{2}+{\mathrm{dtheta}_{l}}^2\,\cos\left(\theta _{l}\right)\,\left(l_{l}-l_{l,d}\right).
$$

$$
a_{r,s}=\mathrm{dds}+\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}+\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)}{2}-\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{2}+\mathrm{ddtheta}_{r}\,l_{r,d}\,\cos\left(\theta _{r}\right)-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,\sin\left(\theta _{l}\right)}{2}+\frac{{\mathrm{dtheta}_{r}}^2\,l_{r}\,\sin\left(\theta _{r}\right)}{2}-{\mathrm{dtheta}_{r}}^2\,l_{r,d}\,\sin\left(\theta _{r}\right).
$$

$$
a_{r,h}=\mathrm{ddtheta}_{r}\,\sin\left(\theta _{r}\right)\,\left(l_{r}-l_{r,d}\right)-\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\sin\left(\theta _{r}\right)}{2}-\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\sin\left(\theta _{l}\right)}{2}-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,\cos\left(\theta _{l}\right)}{2}-\frac{{\mathrm{dtheta}_{r}}^2\,l_{r}\,\cos\left(\theta _{r}\right)}{2}+{\mathrm{dtheta}_{r}}^2\,\cos\left(\theta _{r}\right)\,\left(l_{r}-l_{r,d}\right).
$$

$$
a_{b,s}=-\frac{l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2}{2}-\frac{l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2}{2}+\mathrm{dds}+\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)}{2}+\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{2}.
$$

$$
a_{b,h}=-\frac{l_{l}\,\cos\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2}{2}-\frac{l_{r}\,\cos\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2}{2}-\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\sin\left(\theta _{l}\right)}{2}-\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\sin\left(\theta _{r}\right)}{2}.
$$

## 广义方程

$$
M(q)\ddot q=B_{\mathrm{control}}(q)u+g(q,\dot q).
$$

未列出的元素严格为零。

### $M$ 的非零元素

$$
\left(M\right)_{1,1}=-\frac{I_{\mathrm{wl}}+I_{\mathrm{wr}}+m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{1,2}=\frac{\mathrm{half}_{\mathrm{track}}\,\left(I_{\mathrm{wl}}-I_{\mathrm{wr}}+m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{1,3}=-\frac{\cos\left(\theta _{l}\right)\,\left(I_{\mathrm{wr}}\,l_{l}-I_{\mathrm{wl}}\,l_{l}+l_{l}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-l_{l}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,l_{l,d}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+l_{l}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-l_{l}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+l_{l}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{1,4}=-\frac{\cos\left(\theta _{r}\right)\,\left(I_{\mathrm{wl}}\,l_{r}-I_{\mathrm{wr}}\,l_{r}+l_{r}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+l_{r}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+l_{r}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-l_{r}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{2,1}=\frac{l_{c}\,\cos\left(\theta _{b}\right)\,\left(I_{\mathrm{wl}}+I_{\mathrm{wr}}+m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{2,2}=-\frac{\mathrm{half}_{\mathrm{track}}\,l_{c}\,\cos\left(\theta _{b}\right)\,\left(I_{\mathrm{wl}}-I_{\mathrm{wr}}+m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{2,3}=\frac{l_{c}\,\cos\left(\theta _{b}\right)\,\cos\left(\theta _{l}\right)\,\left(I_{\mathrm{wr}}\,l_{l}-I_{\mathrm{wl}}\,l_{l}-l_{l}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,l_{l,d}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+l_{l}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-l_{l}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+l_{l}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{l_{c}\,l_{l}\,m_{b}\,\sin\left(\theta _{b}\right)\,\sin\left(\theta _{l}\right)}{2}.
$$

$$
\left(M\right)_{2,4}=\frac{l_{c}\,\cos\left(\theta _{b}\right)\,\cos\left(\theta _{r}\right)\,\left(I_{\mathrm{wl}}\,l_{r}-I_{\mathrm{wr}}\,l_{r}+l_{r}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+l_{r}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-l_{r}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{l_{c}\,l_{r}\,m_{b}\,\sin\left(\theta _{b}\right)\,\sin\left(\theta _{r}\right)}{2}.
$$

$$
\left(M\right)_{2,5}=-I_{b}.
$$

$$
\left(M\right)_{3,1}=\frac{\cos\left(\theta _{l}\right)\,\left(I_{\mathrm{wl}}\,l_{l}+l_{l}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-l_{l,d}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+l_{l}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{3,2}=-\frac{\mathrm{half}_{\mathrm{track}}\,\cos\left(\theta _{l}\right)\,\left(I_{\mathrm{wl}}\,l_{l}+l_{l}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-l_{l,d}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+l_{l}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{3,3}=\frac{3\,l_{l}\,l_{l,d}\,m_{l}\,{\cos\left(\theta _{l}\right)}^2}{2}-\frac{{l_{l}}^2\,m_{l}\,{\cos\left(\theta _{l}\right)}^2}{2}-{l_{l,d}}^2\,m_{l}\,{\cos\left(\theta _{l}\right)}^2-\frac{{l_{l}}^2\,m_{\mathrm{wl}}\,{\cos\left(\theta _{l}\right)}^2}{2}-\frac{{l_{l}}^2\,m_{b}\,{\sin\left(\theta _{l}\right)}^2}{4}-\frac{{l_{l}}^2\,m_{l}\,{\sin\left(\theta _{l}\right)}^2}{4}-{l_{l,d}}^2\,m_{l}\,{\sin\left(\theta _{l}\right)}^2-\frac{{l_{l}}^2\,m_{r}\,{\sin\left(\theta _{l}\right)}^2}{4}-\frac{I_{\mathrm{wl}}\,{l_{l}}^2\,{\cos\left(\theta _{l}\right)}^2}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}-I_{l}+l_{l}\,l_{l,d}\,m_{l}\,{\sin\left(\theta _{l}\right)}^2.
$$

$$
\left(M\right)_{3,4}=\frac{l_{l}\,l_{r}\,m_{l}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2}-\frac{l_{l,d}\,l_{r}\,m_{l}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2}+\frac{l_{l}\,l_{r}\,m_{\mathrm{wl}}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2}-\frac{l_{l}\,l_{r}\,m_{b}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}+\frac{l_{l}\,l_{r}\,m_{l}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}-\frac{l_{l,d}\,l_{r}\,m_{l}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2}+\frac{l_{l}\,l_{r}\,m_{r}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}-\frac{l_{l}\,l_{r,d}\,m_{r}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2}+\frac{I_{\mathrm{wl}}\,l_{l}\,l_{r}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{4,1}=\frac{\cos\left(\theta _{r}\right)\,\left(I_{\mathrm{wr}}\,l_{r}+l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+l_{r}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{4,2}=\frac{\mathrm{half}_{\mathrm{track}}\,\cos\left(\theta _{r}\right)\,\left(I_{\mathrm{wr}}\,l_{r}+l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+l_{r}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{4,3}=\frac{l_{l}\,l_{r}\,m_{r}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2}-\frac{l_{l}\,l_{r,d}\,m_{r}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2}+\frac{l_{l}\,l_{r}\,m_{\mathrm{wr}}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2}-\frac{l_{l}\,l_{r}\,m_{b}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}+\frac{l_{l}\,l_{r}\,m_{l}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}-\frac{l_{l,d}\,l_{r}\,m_{l}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2}+\frac{l_{l}\,l_{r}\,m_{r}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}-\frac{l_{l}\,l_{r,d}\,m_{r}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2}+\frac{I_{\mathrm{wr}}\,l_{l}\,l_{r}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{4,4}=\frac{3\,l_{r}\,l_{r,d}\,m_{r}\,{\cos\left(\theta _{r}\right)}^2}{2}-\frac{{l_{r}}^2\,m_{r}\,{\cos\left(\theta _{r}\right)}^2}{2}-{l_{r,d}}^2\,m_{r}\,{\cos\left(\theta _{r}\right)}^2-\frac{{l_{r}}^2\,m_{\mathrm{wr}}\,{\cos\left(\theta _{r}\right)}^2}{2}-\frac{{l_{r}}^2\,m_{b}\,{\sin\left(\theta _{r}\right)}^2}{4}-\frac{{l_{r}}^2\,m_{l}\,{\sin\left(\theta _{r}\right)}^2}{4}-\frac{{l_{r}}^2\,m_{r}\,{\sin\left(\theta _{r}\right)}^2}{4}-{l_{r,d}}^2\,m_{r}\,{\sin\left(\theta _{r}\right)}^2-\frac{I_{\mathrm{wr}}\,{l_{r}}^2\,{\cos\left(\theta _{r}\right)}^2}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}-I_{r}+l_{r}\,l_{r,d}\,m_{r}\,{\sin\left(\theta _{r}\right)}^2.
$$

$$
\left(M\right)_{5,1}=\frac{\mathrm{half}_{\mathrm{track}}\,\left(I_{\mathrm{wl}}-I_{\mathrm{wr}}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{5,2}=-\frac{I_{\mathrm{wl}}\,{\mathrm{half}_{\mathrm{track}}}^2+I_{\mathrm{wr}}\,{\mathrm{half}_{\mathrm{track}}}^2+I_{\mathrm{yaw}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{5,3}=-\frac{\mathrm{half}_{\mathrm{track}}\,l_{l}\,\cos\left(\theta _{l}\right)\,\left(I_{\mathrm{wl}}+I_{\mathrm{wr}}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{5,4}=\frac{\mathrm{half}_{\mathrm{track}}\,l_{r}\,\cos\left(\theta _{r}\right)\,\left(I_{\mathrm{wl}}+I_{\mathrm{wr}}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

### $B_{\mathrm{control}}$ 的非零元素

$$
\left(B_{\mathrm{control}}\right)_{1,1}=-\frac{1}{\mathrm{wheel}_{\mathrm{radius}}}.
$$

$$
\left(B_{\mathrm{control}}\right)_{1,2}=-\frac{1}{\mathrm{wheel}_{\mathrm{radius}}}.
$$

$$
\left(B_{\mathrm{control}}\right)_{2,1}=\frac{l_{c}\,\cos\left(\theta _{b}\right)}{\mathrm{wheel}_{\mathrm{radius}}}.
$$

$$
\left(B_{\mathrm{control}}\right)_{2,2}=\frac{l_{c}\,\cos\left(\theta _{b}\right)}{\mathrm{wheel}_{\mathrm{radius}}}.
$$

$$
\left(B_{\mathrm{control}}\right)_{2,3}=1.
$$

$$
\left(B_{\mathrm{control}}\right)_{2,4}=1.
$$

$$
\left(B_{\mathrm{control}}\right)_{3,1}=\frac{l_{l}\,\cos\left(\theta _{l}\right)}{\mathrm{wheel}_{\mathrm{radius}}}+1.
$$

$$
\left(B_{\mathrm{control}}\right)_{3,3}=-1.
$$

$$
\left(B_{\mathrm{control}}\right)_{4,2}=\frac{l_{r}\,\cos\left(\theta _{r}\right)}{\mathrm{wheel}_{\mathrm{radius}}}+1.
$$

$$
\left(B_{\mathrm{control}}\right)_{4,4}=-1.
$$

$$
\left(B_{\mathrm{control}}\right)_{5,1}=\frac{\mathrm{half}_{\mathrm{track}}}{\mathrm{wheel}_{\mathrm{radius}}}.
$$

$$
\left(B_{\mathrm{control}}\right)_{5,2}=-\frac{\mathrm{half}_{\mathrm{track}}}{\mathrm{wheel}_{\mathrm{radius}}}.
$$

### $g$ 的非零元素

$$
\left(g\right)_{1,1}=\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{l}\,\sin\left(\theta _{l}\right)}{2}-\frac{{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{b}\,\sin\left(\theta _{r}\right)}{2}-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{b}\,\sin\left(\theta _{l}\right)}{2}-{\mathrm{dtheta}_{l}}^2\,l_{l,d}\,m_{l}\,\sin\left(\theta _{l}\right)-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{r}\,\sin\left(\theta _{l}\right)}{2}-\frac{{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{l}\,\sin\left(\theta _{r}\right)}{2}+\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{\mathrm{wl}}\,\sin\left(\theta _{l}\right)}{2}-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{\mathrm{wr}}\,\sin\left(\theta _{l}\right)}{2}+\frac{{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{r}\,\sin\left(\theta _{r}\right)}{2}-{\mathrm{dtheta}_{r}}^2\,l_{r,d}\,m_{r}\,\sin\left(\theta _{r}\right)-\frac{{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{\mathrm{wl}}\,\sin\left(\theta _{r}\right)}{2}+\frac{{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{\mathrm{wr}}\,\sin\left(\theta _{r}\right)}{2}+\frac{I_{\mathrm{wl}}\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,\sin\left(\theta _{l}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{I_{\mathrm{wr}}\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,\sin\left(\theta _{l}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{I_{\mathrm{wl}}\,{\mathrm{dtheta}_{r}}^2\,l_{r}\,\sin\left(\theta _{r}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}+\frac{I_{\mathrm{wr}}\,{\mathrm{dtheta}_{r}}^2\,l_{r}\,\sin\left(\theta _{r}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(g\right)_{2,1}=\frac{l_{c}\,\left(I_{\mathrm{wr}}\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,\cos\left(\theta _{b}\right)\,\sin\left(\theta _{l}\right)-I_{\mathrm{wl}}\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,\cos\left(\theta _{b}\right)\,\sin\left(\theta _{l}\right)-2\,g\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{b}\right)+I_{\mathrm{wl}}\,{\mathrm{dtheta}_{r}}^2\,l_{r}\,\cos\left(\theta _{b}\right)\,\sin\left(\theta _{r}\right)-I_{\mathrm{wr}}\,{\mathrm{dtheta}_{r}}^2\,l_{r}\,\cos\left(\theta _{b}\right)\,\sin\left(\theta _{r}\right)+{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{b}\right)+{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{b}\right)-{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{b}\right)\,\sin\left(\theta _{l}\right)+2\,{\mathrm{dtheta}_{l}}^2\,l_{l,d}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{b}\right)\,\sin\left(\theta _{l}\right)+{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{b}\right)\,\sin\left(\theta _{l}\right)+{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{b}\right)\,\sin\left(\theta _{r}\right)-{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{b}\right)\,\sin\left(\theta _{l}\right)+{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{b}\right)\,\sin\left(\theta _{l}\right)-{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{b}\right)\,\sin\left(\theta _{r}\right)+2\,{\mathrm{dtheta}_{r}}^2\,l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{b}\right)\,\sin\left(\theta _{r}\right)+{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{b}\right)\,\sin\left(\theta _{r}\right)-{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{b}\right)\,\sin\left(\theta _{r}\right)\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(g\right)_{3,1}=\frac{4\,g\,l_{l}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{l}\right)-4\,g\,l_{l}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{l}\right)-2\,I_{\mathrm{wl}}\,{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,\sin\left(2\,\theta _{l}\right)-8\,g\,l_{l,d}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{l}\right)-4\,g\,l_{l}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{l}\right)+{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(2\,\theta _{l}\right)-{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(2\,\theta _{l}\right)+{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(2\,\theta _{l}\right)-2\,{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(2\,\theta _{l}\right)+4\,I_{\mathrm{wl}}\,{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)+2\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{l,d}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(2\,\theta _{l}\right)+2\,{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)+4\,{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)-2\,{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)-4\,{\mathrm{dtheta}_{r}}^2\,l_{l,d}\,l_{r}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)+4\,{\mathrm{dtheta}_{r}}^2\,l_{l,d}\,l_{r}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)-2\,{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)+4\,{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)+4\,{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{8\,{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(g\right)_{4,1}=\frac{4\,g\,l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{r}\right)-4\,g\,l_{r}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{r}\right)-4\,g\,l_{r}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{r}\right)-2\,I_{\mathrm{wr}}\,{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,\sin\left(2\,\theta _{r}\right)-8\,g\,l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(\theta _{r}\right)+{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(2\,\theta _{r}\right)+{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(2\,\theta _{r}\right)-{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(2\,\theta _{r}\right)-2\,{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(2\,\theta _{r}\right)+4\,I_{\mathrm{wr}}\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)+2\,{\mathrm{dtheta}_{r}}^2\,l_{r}\,l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\sin\left(2\,\theta _{r}\right)+2\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)-2\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)+4\,{\mathrm{dtheta}_{l}}^2\,l_{l,d}\,l_{r}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)-2\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)+4\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)+4\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)-4\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)+4\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)}{8\,{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(g\right)_{5,1}=-\frac{\mathrm{half}_{\mathrm{track}}\,\left({\mathrm{dtheta}_{l}}^2\,l_{l}\,\sin\left(\theta _{l}\right)-{\mathrm{dtheta}_{r}}^2\,l_{r}\,\sin\left(\theta _{r}\right)\right)\,\left(I_{\mathrm{wl}}+I_{\mathrm{wr}}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

## 线性化边界

控制器矩阵 `A`、`B` 不是在零输入处单独导出的符号式，而是 `compute_lqr_and_export.m` 在每个静态平衡点 `(x_ref,u0)` 对完整非线性状态方程求数值雅可比。因此保留了 `B(q)u0` 的姿态耦合。
