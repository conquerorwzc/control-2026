# 双闭环等效腿模型：符号动力学 M、B、g 与约束

## 状态和输入合同

状态顺序：`[s, s_dot, phi, phi_dot, theta_L, theta_L_dot, theta_R, theta_R_dot, theta_b, theta_b_dot]`。

输入顺序（模型正方向）：`[Tp_R, Tp_L, Tw_R, Tw_L]`。MCU 输入方向在数值 LQR 脚本中另行映射。

$$
s=s_b-s_{b,0}.
$$

s 是 WBR body 版机身纵向二维坐标，不做 Yaw 的世界系投影。当前模型以 O 为机身参考点且 body_com_to_pitch_axis=0，因此固件 O 里程计可读取相同数值。模型假设：平地、纯滚动、两轮接地、无 Roll、固定腿长工作点。左右地面对轮法向力相等用于闭合竖直内力；这是降阶模型前提，不是运行时接触力判断。

Tw 正值定义为使对应轮向前滚动的**轮端力矩**。轮对腿的反作用力矩因此为 `-Tw`；此处定义不等于未来 H6215 电机轴正指令，后者仍需独立标定。

测量端髋点速度可包含腿长变化项；本符号动力学的单个工作点不把 `l_dot/l_ddot` 纳入状态。

## 原始 Newton-Euler 方程

以下 17 条残差包含机体、左右腿、左右轮、Yaw 方程和等法向力闭合。程序先消去 12 个接触内力，再得到五条广义坐标方程；不维护手写的 `eq1` 至 `eq5`。

$$
r_{1}=F_{l,\mathrm{to},b,h}+F_{r,\mathrm{to},b,h}-a_{b,h}\,m_{b}.
$$

$$
r_{2}=F_{l,\mathrm{to},b,v}+F_{r,\mathrm{to},b,v}-a_{b,v}\,m_{b}+g\,m_{b}.
$$

$$
r_{3}=T_{p,l}+T_{p,r}-I_{b}\,\mathrm{ddtheta}_{b}+g\,l_{b}\,m_{b}\,\sin\left(\theta _{b}+\theta _{\mathrm{b0}}\right).
$$

$$
r_{4}=F_{\mathrm{wl},\mathrm{to},l,h}-F_{l,\mathrm{to},b,h}-a_{l,h}\,m_{l}.
$$

$$
r_{5}=F_{\mathrm{wl},\mathrm{to},l,v}-F_{l,\mathrm{to},b,v}-a_{l,v}\,m_{l}+g\,m_{l}.
$$

$$
r_{6}=T_{p,l}-T_{w,l}-I_{l}\,\mathrm{ddtheta}_{l}-\cos\left(\theta _{l}\right)\,\left(F_{\mathrm{wl},\mathrm{to},l,h}\,l_{l,d}+F_{l,\mathrm{to},b,h}\,\left(l_{l}-l_{l,d}\right)\right)+\sin\left(\theta _{l}\right)\,\left(F_{\mathrm{wl},\mathrm{to},l,v}\,l_{l,d}+F_{l,\mathrm{to},b,v}\,\left(l_{l}-l_{l,d}\right)\right).
$$

$$
r_{7}=F_{\mathrm{wr},\mathrm{to},r,h}-F_{r,\mathrm{to},b,h}-a_{r,h}\,m_{r}.
$$

$$
r_{8}=F_{\mathrm{wr},\mathrm{to},r,v}-F_{r,\mathrm{to},b,v}-a_{r,v}\,m_{r}+g\,m_{r}.
$$

$$
r_{9}=T_{p,r}-T_{w,r}-I_{r}\,\mathrm{ddtheta}_{r}-\cos\left(\theta _{r}\right)\,\left(F_{\mathrm{wr},\mathrm{to},r,h}\,l_{r,d}+F_{r,\mathrm{to},b,h}\,\left(l_{r}-l_{r,d}\right)\right)+\sin\left(\theta _{r}\right)\,\left(F_{\mathrm{wr},\mathrm{to},r,v}\,l_{r,d}+F_{r,\mathrm{to},b,v}\,\left(l_{r}-l_{r,d}\right)\right).
$$

$$
r_{10}=F_{g,\mathrm{to},\mathrm{wl},h}-F_{\mathrm{wl},\mathrm{to},l,h}-a_{\mathrm{wl},h}\,m_{\mathrm{wl}}.
$$

$$
r_{11}=F_{g,\mathrm{to},\mathrm{wl},v}-F_{\mathrm{wl},\mathrm{to},l,v}-a_{\mathrm{wl},v}\,m_{\mathrm{wl}}+g\,m_{\mathrm{wl}}.
$$

$$
r_{12}=T_{w,l}-I_{\mathrm{wl}}\,\mathrm{ddtheta}_{\mathrm{wl}}-F_{g,\mathrm{to},\mathrm{wl},h}\,\mathrm{wheel}_{\mathrm{radius}}.
$$

$$
r_{13}=F_{g,\mathrm{to},\mathrm{wr},h}-F_{\mathrm{wr},\mathrm{to},r,h}-a_{\mathrm{wr},h}\,m_{\mathrm{wr}}.
$$

$$
r_{14}=F_{g,\mathrm{to},\mathrm{wr},v}-F_{\mathrm{wr},\mathrm{to},r,v}-a_{\mathrm{wr},v}\,m_{\mathrm{wr}}+g\,m_{\mathrm{wr}}.
$$

$$
r_{15}=T_{w,r}-I_{\mathrm{wr}}\,\mathrm{ddtheta}_{\mathrm{wr}}-F_{g,\mathrm{to},\mathrm{wr},h}\,\mathrm{wheel}_{\mathrm{radius}}.
$$

$$
r_{16}=-I_{\mathrm{yaw}}\,\mathrm{ddphi}-\mathrm{half}_{\mathrm{track}}\,\left(F_{g,\mathrm{to},\mathrm{wl},h}-F_{g,\mathrm{to},\mathrm{wr},h}\right).
$$

$$
r_{17}=F_{g,\mathrm{to},\mathrm{wl},v}-F_{g,\mathrm{to},\mathrm{wr},v}.
$$

## 自动消元后的五条广义方程

$$
E_{1}=\frac{T_{w,l}}{\mathrm{wheel}_{\mathrm{radius}}}-\mathrm{dds}\,m_{l}-\mathrm{dds}\,m_{r}-\mathrm{dds}\,m_{\mathrm{wl}}-\mathrm{dds}\,m_{\mathrm{wr}}-\mathrm{dds}\,m_{b}+\frac{T_{w,r}}{\mathrm{wheel}_{\mathrm{radius}}}+\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,m_{l}-\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,m_{r}+\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,m_{\mathrm{wl}}-\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,m_{\mathrm{wr}}-\frac{I_{\mathrm{wl}}\,\mathrm{dds}}{{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{I_{\mathrm{wr}}\,\mathrm{dds}}{{\mathrm{wheel}_{\mathrm{radius}}}^2}+\frac{I_{\mathrm{wl}}\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}}{{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{I_{\mathrm{wr}}\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}}{{\mathrm{wheel}_{\mathrm{radius}}}^2}+\mathrm{ddtheta}_{l}\,l_{l}\,m_{l}\,\cos\left(\theta _{l}\right)-\mathrm{ddtheta}_{l}\,l_{l,d}\,m_{l}\,\cos\left(\theta _{l}\right)+\mathrm{ddtheta}_{l}\,l_{l}\,m_{\mathrm{wl}}\,\cos\left(\theta _{l}\right)+\mathrm{ddtheta}_{r}\,l_{r}\,m_{r}\,\cos\left(\theta _{r}\right)-\mathrm{ddtheta}_{r}\,l_{r,d}\,m_{r}\,\cos\left(\theta _{r}\right)+\mathrm{ddtheta}_{r}\,l_{r}\,m_{\mathrm{wr}}\,\cos\left(\theta _{r}\right)-{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{l}\,\sin\left(\theta _{l}\right)+{\mathrm{dtheta}_{l}}^2\,l_{l,d}\,m_{l}\,\sin\left(\theta _{l}\right)-{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{\mathrm{wl}}\,\sin\left(\theta _{l}\right)-{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{r}\,\sin\left(\theta _{r}\right)+{\mathrm{dtheta}_{r}}^2\,l_{r,d}\,m_{r}\,\sin\left(\theta _{r}\right)-{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{\mathrm{wr}}\,\sin\left(\theta _{r}\right)+\frac{I_{\mathrm{wl}}\,\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}+\frac{I_{\mathrm{wr}}\,\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{I_{\mathrm{wl}}\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,\sin\left(\theta _{l}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{I_{\mathrm{wr}}\,{\mathrm{dtheta}_{r}}^2\,l_{r}\,\sin\left(\theta _{r}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
E_{2}=T_{p,l}+T_{p,r}-I_{b}\,\mathrm{ddtheta}_{b}+g\,l_{b}\,m_{b}\,\sin\left(\theta _{b}+\theta _{\mathrm{b0}}\right).
$$

$$
E_{3}=T_{p,l}-T_{w,l}-I_{l}\,\mathrm{ddtheta}_{l}-\mathrm{ddtheta}_{l}\,{l_{l}}^2\,m_{l}\,{\cos\left(\theta _{l}\right)}^2-\mathrm{ddtheta}_{l}\,{l_{l,d}}^2\,m_{l}\,{\cos\left(\theta _{l}\right)}^2-\mathrm{ddtheta}_{l}\,{l_{l}}^2\,m_{\mathrm{wl}}\,{\cos\left(\theta _{l}\right)}^2-\frac{\mathrm{ddtheta}_{l}\,{l_{l}}^2\,m_{b}\,{\sin\left(\theta _{l}\right)}^2}{4}-\mathrm{ddtheta}_{l}\,{l_{l,d}}^2\,m_{l}\,{\sin\left(\theta _{l}\right)}^2+\mathrm{dds}\,l_{l}\,m_{l}\,\cos\left(\theta _{l}\right)-\mathrm{dds}\,l_{l,d}\,m_{l}\,\cos\left(\theta _{l}\right)+\mathrm{dds}\,l_{l}\,m_{\mathrm{wl}}\,\cos\left(\theta _{l}\right)-\frac{g\,l_{l}\,m_{b}\,\sin\left(\theta _{l}\right)}{2}+\frac{g\,l_{l}\,m_{l}\,\sin\left(\theta _{l}\right)}{2}-g\,l_{l,d}\,m_{l}\,\sin\left(\theta _{l}\right)-\frac{g\,l_{l}\,m_{r}\,\sin\left(\theta _{l}\right)}{2}+\frac{g\,l_{l}\,m_{\mathrm{wl}}\,\sin\left(\theta _{l}\right)}{2}-\frac{g\,l_{l}\,m_{\mathrm{wr}}\,\sin\left(\theta _{l}\right)}{2}-\frac{{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{b}\,\sin\left(2\,\theta _{l}\right)}{8}+\frac{{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{l}\,\sin\left(2\,\theta _{l}\right)}{2}+\frac{{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{\mathrm{wl}}\,\sin\left(2\,\theta _{l}\right)}{2}-\frac{T_{w,l}\,l_{l}\,\cos\left(\theta _{l}\right)}{\mathrm{wheel}_{\mathrm{radius}}}+\frac{I_{\mathrm{wl}}\,\mathrm{dds}\,l_{l}\,\cos\left(\theta _{l}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}+2\,\mathrm{ddtheta}_{l}\,l_{l}\,l_{l,d}\,m_{l}\,{\cos\left(\theta _{l}\right)}^2+\frac{\mathrm{ddtheta}_{l}\,l_{l}\,l_{l,d}\,m_{l}\,{\sin\left(\theta _{l}\right)}^2}{2}+\frac{I_{\mathrm{wl}}\,{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,\sin\left(2\,\theta _{l}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{3\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{l,d}\,m_{l}\,\sin\left(2\,\theta _{l}\right)}{4}-\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,l_{l}\,m_{l}\,\cos\left(\theta _{l}\right)+\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,l_{l,d}\,m_{l}\,\cos\left(\theta _{l}\right)-\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,l_{l}\,m_{\mathrm{wl}}\,\cos\left(\theta _{l}\right)-\frac{I_{\mathrm{wl}}\,\mathrm{ddtheta}_{l}\,{l_{l}}^2\,{\cos\left(\theta _{l}\right)}^2}{{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,m_{b}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)}{4}-\frac{{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r,d}\,m_{r}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)}{2}-\frac{I_{\mathrm{wl}}\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,l_{l}\,\cos\left(\theta _{l}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{\mathrm{ddtheta}_{r}\,l_{l}\,l_{r}\,m_{b}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}-\frac{\mathrm{ddtheta}_{r}\,l_{l}\,l_{r,d}\,m_{r}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2}.
$$

$$
E_{4}=T_{p,r}-T_{w,r}-I_{r}\,\mathrm{ddtheta}_{r}-\mathrm{ddtheta}_{r}\,{l_{r}}^2\,m_{r}\,{\cos\left(\theta _{r}\right)}^2-\mathrm{ddtheta}_{r}\,{l_{r,d}}^2\,m_{r}\,{\cos\left(\theta _{r}\right)}^2-\mathrm{ddtheta}_{r}\,{l_{r}}^2\,m_{\mathrm{wr}}\,{\cos\left(\theta _{r}\right)}^2-\frac{\mathrm{ddtheta}_{r}\,{l_{r}}^2\,m_{b}\,{\sin\left(\theta _{r}\right)}^2}{4}-\mathrm{ddtheta}_{r}\,{l_{r,d}}^2\,m_{r}\,{\sin\left(\theta _{r}\right)}^2+\mathrm{dds}\,l_{r}\,m_{r}\,\cos\left(\theta _{r}\right)-\mathrm{dds}\,l_{r,d}\,m_{r}\,\cos\left(\theta _{r}\right)+\mathrm{dds}\,l_{r}\,m_{\mathrm{wr}}\,\cos\left(\theta _{r}\right)-\frac{g\,l_{r}\,m_{b}\,\sin\left(\theta _{r}\right)}{2}-\frac{g\,l_{r}\,m_{l}\,\sin\left(\theta _{r}\right)}{2}+\frac{g\,l_{r}\,m_{r}\,\sin\left(\theta _{r}\right)}{2}-g\,l_{r,d}\,m_{r}\,\sin\left(\theta _{r}\right)-\frac{g\,l_{r}\,m_{\mathrm{wl}}\,\sin\left(\theta _{r}\right)}{2}+\frac{g\,l_{r}\,m_{\mathrm{wr}}\,\sin\left(\theta _{r}\right)}{2}-\frac{{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{b}\,\sin\left(2\,\theta _{r}\right)}{8}+\frac{{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{r}\,\sin\left(2\,\theta _{r}\right)}{2}+\frac{{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{\mathrm{wr}}\,\sin\left(2\,\theta _{r}\right)}{2}-\frac{T_{w,r}\,l_{r}\,\cos\left(\theta _{r}\right)}{\mathrm{wheel}_{\mathrm{radius}}}+\frac{I_{\mathrm{wr}}\,\mathrm{dds}\,l_{r}\,\cos\left(\theta _{r}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}+2\,\mathrm{ddtheta}_{r}\,l_{r}\,l_{r,d}\,m_{r}\,{\cos\left(\theta _{r}\right)}^2+\frac{\mathrm{ddtheta}_{r}\,l_{r}\,l_{r,d}\,m_{r}\,{\sin\left(\theta _{r}\right)}^2}{2}+\frac{I_{\mathrm{wr}}\,{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,\sin\left(2\,\theta _{r}\right)}{2\,{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{3\,{\mathrm{dtheta}_{r}}^2\,l_{r}\,l_{r,d}\,m_{r}\,\sin\left(2\,\theta _{r}\right)}{4}+\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,l_{r}\,m_{r}\,\cos\left(\theta _{r}\right)-\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,l_{r,d}\,m_{r}\,\cos\left(\theta _{r}\right)+\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,l_{r}\,m_{\mathrm{wr}}\,\cos\left(\theta _{r}\right)-\frac{I_{\mathrm{wr}}\,\mathrm{ddtheta}_{r}\,{l_{r}}^2\,{\cos\left(\theta _{r}\right)}^2}{{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,m_{b}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l,d}\,l_{r}\,m_{l}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2}+\frac{I_{\mathrm{wr}}\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}\,l_{r}\,\cos\left(\theta _{r}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{\mathrm{ddtheta}_{l}\,l_{l}\,l_{r}\,m_{b}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4}-\frac{\mathrm{ddtheta}_{l}\,l_{l,d}\,l_{r}\,m_{l}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{2}.
$$

$$
E_{5}=-I_{\mathrm{yaw}}\,\mathrm{ddphi}-\mathrm{half}_{\mathrm{track}}\,\left(\frac{-I_{\mathrm{wl}}\,l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2-I_{\mathrm{wl}}\,\mathrm{dds}+T_{w,l}\,\mathrm{wheel}_{\mathrm{radius}}+I_{\mathrm{wl}}\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}+I_{\mathrm{wl}}\,\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}+\frac{I_{\mathrm{wr}}\,l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2+I_{\mathrm{wr}}\,\mathrm{dds}-T_{w,r}\,\mathrm{wheel}_{\mathrm{radius}}+I_{\mathrm{wr}}\,\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}-I_{\mathrm{wr}}\,\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}\right).
$$

## Body 版 s_b 运动学代换

对每一侧：

$$
R\theta_{w,L}=s_b-b\phi-l_L\sin\theta_L,\qquad R\theta_{w,R}=s_b+b\phi-l_R\sin\theta_R.
$$

$$
s_b=\frac{R\theta_{w,L}+l_L\sin\theta_L+R\theta_{w,R}+l_R\sin\theta_R}{2}.
$$

以下为固定腿长工作点所用的加速度代换：

$$
\mathrm{ddtheta}_{\mathrm{wl}}=\frac{l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2+\mathrm{dds}-\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}-\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)}{\mathrm{wheel}_{\mathrm{radius}}}.
$$

$$
\mathrm{ddtheta}_{\mathrm{wr}}=\frac{l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2+\mathrm{dds}+\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}-\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{\mathrm{wheel}_{\mathrm{radius}}}.
$$

$$
a_{\mathrm{wl},h}=l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2+\mathrm{dds}-\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}-\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right).
$$

$$
a_{\mathrm{wl},v}=0.
$$

$$
a_{\mathrm{wr},h}=l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2+\mathrm{dds}+\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}-\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right).
$$

$$
a_{\mathrm{wr},v}=0.
$$

$$
a_{l,h}=\mathrm{dds}-\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}-\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)+\mathrm{ddtheta}_{l}\,l_{l,d}\,\cos\left(\theta _{l}\right)+{\mathrm{dtheta}_{l}}^2\,l_{l}\,\sin\left(\theta _{l}\right)-{\mathrm{dtheta}_{l}}^2\,l_{l,d}\,\sin\left(\theta _{l}\right).
$$

$$
a_{l,v}=-l_{l,d}\,\cos\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2-\mathrm{ddtheta}_{l}\,l_{l,d}\,\sin\left(\theta _{l}\right).
$$

$$
a_{r,h}=\mathrm{dds}+\mathrm{ddphi}\,\mathrm{half}_{\mathrm{track}}-\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)+\mathrm{ddtheta}_{r}\,l_{r,d}\,\cos\left(\theta _{r}\right)+{\mathrm{dtheta}_{r}}^2\,l_{r}\,\sin\left(\theta _{r}\right)-{\mathrm{dtheta}_{r}}^2\,l_{r,d}\,\sin\left(\theta _{r}\right).
$$

$$
a_{r,v}=-l_{r,d}\,\cos\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2-\mathrm{ddtheta}_{r}\,l_{r,d}\,\sin\left(\theta _{r}\right).
$$

$$
a_{b,h}=\mathrm{dds}.
$$

$$
a_{b,v}=-\frac{l_{l}\,\cos\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2}{2}-\frac{l_{r}\,\cos\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2}{2}-\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\sin\left(\theta _{l}\right)}{2}-\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\sin\left(\theta _{r}\right)}{2}.
$$

## 广义方程

$$
M(q)\ddot q=B_{\mathrm{control}}(q)u+g(q,\dot q).
$$

未列出的元素严格为零。

### $M$ 的非零元素

$$
\left(M\right)_{1,1}=-m_{b}-m_{l}-m_{r}-m_{\mathrm{wl}}-m_{\mathrm{wr}}-\frac{I_{\mathrm{wl}}}{{\mathrm{wheel}_{\mathrm{radius}}}^2}-\frac{I_{\mathrm{wr}}}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{1,2}=\frac{\mathrm{half}_{\mathrm{track}}\,\left(I_{\mathrm{wl}}-I_{\mathrm{wr}}+m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{1,3}=\frac{\cos\left(\theta _{l}\right)\,\left(I_{\mathrm{wl}}\,l_{l}+l_{l}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-l_{l,d}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+l_{l}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{1,4}=\frac{\cos\left(\theta _{r}\right)\,\left(I_{\mathrm{wr}}\,l_{r}+l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+l_{r}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
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
\left(M\right)_{3,3}=2\,l_{l}\,l_{l,d}\,m_{l}\,{\cos\left(\theta _{l}\right)}^2-{l_{l}}^2\,m_{l}\,{\cos\left(\theta _{l}\right)}^2-{l_{l,d}}^2\,m_{l}\,{\cos\left(\theta _{l}\right)}^2-{l_{l}}^2\,m_{\mathrm{wl}}\,{\cos\left(\theta _{l}\right)}^2-\frac{{l_{l}}^2\,m_{b}\,{\sin\left(\theta _{l}\right)}^2}{4}-{l_{l,d}}^2\,m_{l}\,{\sin\left(\theta _{l}\right)}^2-\frac{I_{\mathrm{wl}}\,{l_{l}}^2\,{\cos\left(\theta _{l}\right)}^2}{{\mathrm{wheel}_{\mathrm{radius}}}^2}-I_{l}+\frac{l_{l}\,l_{l,d}\,m_{l}\,{\sin\left(\theta _{l}\right)}^2}{2}.
$$

$$
\left(M\right)_{3,4}=-\frac{l_{l}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)\,\left(l_{r}\,m_{b}+2\,l_{r,d}\,m_{r}\right)}{4}.
$$

$$
\left(M\right)_{4,1}=\frac{\cos\left(\theta _{r}\right)\,\left(I_{\mathrm{wr}}\,l_{r}+l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+l_{r}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{4,2}=\frac{\mathrm{half}_{\mathrm{track}}\,\cos\left(\theta _{r}\right)\,\left(I_{\mathrm{wr}}\,l_{r}+l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+l_{r}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{4,3}=-\frac{l_{r}\,\sin\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)\,\left(l_{l}\,m_{b}+2\,l_{l,d}\,m_{l}\right)}{4}.
$$

$$
\left(M\right)_{4,4}=2\,l_{r}\,l_{r,d}\,m_{r}\,{\cos\left(\theta _{r}\right)}^2-{l_{r}}^2\,m_{r}\,{\cos\left(\theta _{r}\right)}^2-{l_{r,d}}^2\,m_{r}\,{\cos\left(\theta _{r}\right)}^2-{l_{r}}^2\,m_{\mathrm{wr}}\,{\cos\left(\theta _{r}\right)}^2-\frac{{l_{r}}^2\,m_{b}\,{\sin\left(\theta _{r}\right)}^2}{4}-{l_{r,d}}^2\,m_{r}\,{\sin\left(\theta _{r}\right)}^2-\frac{I_{\mathrm{wr}}\,{l_{r}}^2\,{\cos\left(\theta _{r}\right)}^2}{{\mathrm{wheel}_{\mathrm{radius}}}^2}-I_{r}+\frac{l_{r}\,l_{r,d}\,m_{r}\,{\sin\left(\theta _{r}\right)}^2}{2}.
$$

$$
\left(M\right)_{5,1}=\frac{\mathrm{half}_{\mathrm{track}}\,\left(I_{\mathrm{wl}}-I_{\mathrm{wr}}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{5,2}=-\frac{I_{\mathrm{wl}}\,{\mathrm{half}_{\mathrm{track}}}^2+I_{\mathrm{wr}}\,{\mathrm{half}_{\mathrm{track}}}^2+I_{\mathrm{yaw}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{5,3}=-\frac{I_{\mathrm{wl}}\,\mathrm{half}_{\mathrm{track}}\,l_{l}\,\cos\left(\theta _{l}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(M\right)_{5,4}=\frac{I_{\mathrm{wr}}\,\mathrm{half}_{\mathrm{track}}\,l_{r}\,\cos\left(\theta _{r}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

### $B_{\mathrm{control}}$ 的非零元素

$$
\left(B_{\mathrm{control}}\right)_{1,3}=-\frac{1}{\mathrm{wheel}_{\mathrm{radius}}}.
$$

$$
\left(B_{\mathrm{control}}\right)_{1,4}=-\frac{1}{\mathrm{wheel}_{\mathrm{radius}}}.
$$

$$
\left(B_{\mathrm{control}}\right)_{2,1}=-1.
$$

$$
\left(B_{\mathrm{control}}\right)_{2,2}=-1.
$$

$$
\left(B_{\mathrm{control}}\right)_{3,2}=-1.
$$

$$
\left(B_{\mathrm{control}}\right)_{3,4}=\frac{l_{l}\,\cos\left(\theta _{l}\right)}{\mathrm{wheel}_{\mathrm{radius}}}+1.
$$

$$
\left(B_{\mathrm{control}}\right)_{4,1}=-1.
$$

$$
\left(B_{\mathrm{control}}\right)_{4,3}=\frac{l_{r}\,\cos\left(\theta _{r}\right)}{\mathrm{wheel}_{\mathrm{radius}}}+1.
$$

$$
\left(B_{\mathrm{control}}\right)_{5,3}=-\frac{\mathrm{half}_{\mathrm{track}}}{\mathrm{wheel}_{\mathrm{radius}}}.
$$

$$
\left(B_{\mathrm{control}}\right)_{5,4}=\frac{\mathrm{half}_{\mathrm{track}}}{\mathrm{wheel}_{\mathrm{radius}}}.
$$

### $g$ 的非零元素

$$
\left(g\right)_{1,1}={\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{l}\,\sin\left(\theta _{l}\right)-{\mathrm{dtheta}_{l}}^2\,l_{l,d}\,m_{l}\,\sin\left(\theta _{l}\right)+{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{\mathrm{wl}}\,\sin\left(\theta _{l}\right)+{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{r}\,\sin\left(\theta _{r}\right)-{\mathrm{dtheta}_{r}}^2\,l_{r,d}\,m_{r}\,\sin\left(\theta _{r}\right)+{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{\mathrm{wr}}\,\sin\left(\theta _{r}\right)+\frac{I_{\mathrm{wl}}\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,\sin\left(\theta _{l}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}+\frac{I_{\mathrm{wr}}\,{\mathrm{dtheta}_{r}}^2\,l_{r}\,\sin\left(\theta _{r}\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(g\right)_{2,1}=-g\,l_{b}\,m_{b}\,\sin\left(\theta _{b}+\theta _{\mathrm{b0}}\right).
$$

$$
\left(g\right)_{3,1}=\frac{\sin\left(\theta _{l}\right)\,\left(2\,g\,l_{l}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-4\,I_{\mathrm{wl}}\,{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,\cos\left(\theta _{l}\right)-2\,g\,l_{l}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+4\,g\,l_{l,d}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,g\,l_{l}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-2\,g\,l_{l}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,g\,l_{l}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)-4\,{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)-4\,{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)+{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)+6\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{l,d}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)+2\,{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)\right)}{4\,{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(g\right)_{4,1}=\frac{\sin\left(\theta _{r}\right)\,\left(2\,g\,l_{r}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-4\,I_{\mathrm{wr}}\,{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,\cos\left(\theta _{r}\right)+2\,g\,l_{r}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-2\,g\,l_{r}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+4\,g\,l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+2\,g\,l_{r}\,m_{\mathrm{wl}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2-2\,g\,l_{r}\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2+{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)-4\,{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)-4\,{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{\mathrm{wr}}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)+{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,m_{b}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)+2\,{\mathrm{dtheta}_{l}}^2\,l_{l,d}\,l_{r}\,m_{l}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{l}\right)+6\,{\mathrm{dtheta}_{r}}^2\,l_{r}\,l_{r,d}\,m_{r}\,{\mathrm{wheel}_{\mathrm{radius}}}^2\,\cos\left(\theta _{r}\right)\right)}{4\,{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

$$
\left(g\right)_{5,1}=-\frac{\mathrm{half}_{\mathrm{track}}\,\left(I_{\mathrm{wl}}\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,\sin\left(\theta _{l}\right)-I_{\mathrm{wr}}\,{\mathrm{dtheta}_{r}}^2\,l_{r}\,\sin\left(\theta _{r}\right)\right)}{{\mathrm{wheel}_{\mathrm{radius}}}^2}.
$$

## 线性化边界

控制器矩阵 `A`、`B` 不是在零输入处单独导出的符号式，而是 `compute_lqr_and_export.m` 在每个静态平衡点 `(x_ref,u0)` 对完整非线性状态方程求数值雅可比。因此保留了 `B(q)u0` 的姿态耦合。
