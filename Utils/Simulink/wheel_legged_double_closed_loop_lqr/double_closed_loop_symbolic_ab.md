# 双闭环等效腿模型：符号动力学 M、B、g 与约束

## 状态和输入合同

状态顺序：`[s, s_dot, phi, phi_dot, theta_L, theta_L_dot, theta_R, theta_R_dot, theta_b, theta_b_dot]`。

输入顺序（模型正方向）：`[Tp_R, Tp_L, Tw_R, Tw_L]`。MCU 输入方向在数值 LQR 脚本中另行映射。

$$
s=x_O-x_{O,0}.
$$

模型假设：平地、纯滚动、两轮接地、无 Roll、固定腿长工作点。`eq3/eq4` 额外采用左右地面对轮法向力相等的闭合假设；这是对称降阶模型的前提，不是运行时接触力判断。

测量端髋点速度可包含腿长变化项；本符号动力学的单个工作点不把 `l_dot/l_ddot` 纳入状态。

## 五条已化简动力学方程

$$
eq_1=a_{b,h}\,m_{b}+a_{l,h}\,m_{l}+a_{r,h}\,m_{r}+a_{\mathrm{wl},h}\,m_{\mathrm{wl}}+a_{\mathrm{wr},h}\,m_{\mathrm{wr}}+\frac{T_{w,l}+T_{w,r}+I_{\mathrm{wl}}\,\mathrm{ddtheta}_{\mathrm{wl}}+I_{\mathrm{wr}}\,\mathrm{ddtheta}_{\mathrm{wr}}}{R}.
$$

$$
eq_2=I_{b}\,\mathrm{ddtheta}_{b}-T_{p,r}-T_{p,l}-g\,l_{b}\,m_{b}\,\sin\left(\theta _{b}+\theta _{\mathrm{b0}}\right).
$$

$$
eq_3=T_{p,r}-T_{w,r}+I_{r}\,\mathrm{ddtheta}_{r}-\frac{l_{r}\,\sin\left(\theta _{r}\right)\,\left(a_{b,v}\,m_{b}+a_{l,v}\,m_{l}+a_{r,v}\,m_{r}\right)}{2}-\frac{l_{r}\,\cos\left(\theta _{r}\right)\,\left(T_{w,r}+I_{\mathrm{wr}}\,\mathrm{ddtheta}_{\mathrm{wr}}\right)}{R}-g\,l_{r,d}\,m_{r}\,\sin\left(\theta _{r}+\theta _{\mathrm{r0}}\right)-a_{\mathrm{wr},h}\,l_{r}\,m_{\mathrm{wr}}\,\cos\left(\theta _{r}\right)+\frac{g\,l_{r}\,\sin\left(\theta _{r}\right)\,\left(m_{b}+m_{l}+m_{r}+m_{\mathrm{wl}}-m_{\mathrm{wr}}\right)}{2}.
$$

$$
eq_4=T_{p,l}-T_{w,l}+I_{l}\,\mathrm{ddtheta}_{l}-\frac{l_{l}\,\sin\left(\theta _{l}\right)\,\left(a_{b,v}\,m_{b}+a_{l,v}\,m_{l}+a_{r,v}\,m_{r}\right)}{2}-\frac{l_{l}\,\cos\left(\theta _{l}\right)\,\left(T_{w,l}+I_{\mathrm{wl}}\,\mathrm{ddtheta}_{\mathrm{wl}}\right)}{R}-g\,l_{l,d}\,m_{l}\,\sin\left(\theta _{l}+\theta _{\mathrm{l0}}\right)-a_{\mathrm{wl},h}\,l_{l}\,m_{\mathrm{wl}}\,\cos\left(\theta _{l}\right)+\frac{g\,l_{l}\,\sin\left(\theta _{l}\right)\,\left(m_{b}+m_{l}+m_{r}-m_{\mathrm{wl}}+m_{\mathrm{wr}}\right)}{2}.
$$

$$
eq_5=I_{\mathrm{yaw}}\,\mathrm{ddphi}-\frac{R_{w}\,\left(T_{w,l}-T_{w,r}+I_{\mathrm{wl}}\,\mathrm{ddtheta}_{\mathrm{wl}}-I_{\mathrm{wr}}\,\mathrm{ddtheta}_{\mathrm{wr}}\right)}{R}.
$$

## 髋点 O 运动学代换

对每一侧：

$$
s_{O,i}=R\theta_{w,i}+l_i\sin\theta_i,
$$

$$
\dot{s}_{O,i}=R\dot{\theta}_{w,i}+\dot l_i\sin\theta_i+l_i\dot{\theta}_i\cos\theta_i.
$$

以下为固定腿长工作点所用的加速度代换：

$$
\mathrm{ddtheta}_{\mathrm{wr}}=\frac{\frac{l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2}{2}+\frac{l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2}{2}+\mathrm{ddX}_{b,h}+R_{w}\,\mathrm{ddphi}-\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)}{2}-\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{2}}{R}.
$$

$$
\mathrm{ddtheta}_{\mathrm{wl}}=\frac{\frac{l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2}{2}+\frac{l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2}{2}+\mathrm{ddX}_{b,h}-R_{w}\,\mathrm{ddphi}-\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)}{2}-\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{2}}{R}.
$$

$$
a_{\mathrm{wr},h}=\frac{l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2}{2}+\frac{l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2}{2}+\mathrm{ddX}_{b,h}+R_{w}\,\mathrm{ddphi}-\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)}{2}-\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{2}.
$$

$$
a_{\mathrm{wl},h}=\frac{l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2}{2}+\frac{l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2}{2}+\mathrm{ddX}_{b,h}-R_{w}\,\mathrm{ddphi}-\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)}{2}-\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{2}.
$$

$$
a_{\mathrm{wr},v}=0.
$$

$$
a_{\mathrm{wl},v}=0.
$$

$$
a_{r,h}=\frac{l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2}{2}-\frac{l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2}{2}+\mathrm{ddX}_{b,h}+R_{w}\,\mathrm{ddphi}-\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)}{2}+\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{2}.
$$

$$
a_{l,h}=-\frac{l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2}{2}+\frac{l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2}{2}+\mathrm{ddX}_{b,h}-R_{w}\,\mathrm{ddphi}+\frac{\mathrm{ddtheta}_{l}\,l_{l}\,\cos\left(\theta _{l}\right)}{2}-\frac{\mathrm{ddtheta}_{r}\,l_{r}\,\cos\left(\theta _{r}\right)}{2}.
$$

$$
a_{r,v}=-l_{r}\,\cos\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2-\mathrm{ddtheta}_{r}\,l_{r}\,\sin\left(\theta _{r}\right).
$$

$$
a_{l,v}=-l_{l}\,\cos\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2-\mathrm{ddtheta}_{l}\,l_{l}\,\sin\left(\theta _{l}\right).
$$

$$
a_{b,h}=\mathrm{ddX}_{b,h}.
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
\left(M\right)_{1,1}=m_{b}+m_{l}+m_{r}+m_{\mathrm{wl}}+m_{\mathrm{wr}}+\frac{2\,I_{\mathrm{wl}}+2\,I_{\mathrm{wr}}}{2\,R^2}.
$$

$$
\left(M\right)_{1,2}=R_{w}\,m_{r}-R_{w}\,m_{l}-R_{w}\,m_{\mathrm{wl}}+R_{w}\,m_{\mathrm{wr}}-\frac{2\,I_{\mathrm{wl}}\,R_{w}-2\,I_{\mathrm{wr}}\,R_{w}}{2\,R^2}.
$$

$$
\left(M\right)_{1,3}=\frac{l_{l}\,m_{l}\,\cos\left(\theta _{l}\right)}{2}-\frac{I_{\mathrm{wl}}\,l_{l}\,\cos\left(\theta _{l}\right)+I_{\mathrm{wr}}\,l_{l}\,\cos\left(\theta _{l}\right)}{2\,R^2}-\frac{l_{l}\,m_{r}\,\cos\left(\theta _{l}\right)}{2}-\frac{l_{l}\,m_{\mathrm{wl}}\,\cos\left(\theta _{l}\right)}{2}-\frac{l_{l}\,m_{\mathrm{wr}}\,\cos\left(\theta _{l}\right)}{2}.
$$

$$
\left(M\right)_{1,4}=\frac{l_{r}\,m_{r}\,\cos\left(\theta _{r}\right)}{2}-\frac{l_{r}\,m_{l}\,\cos\left(\theta _{r}\right)}{2}-\frac{I_{\mathrm{wl}}\,l_{r}\,\cos\left(\theta _{r}\right)+I_{\mathrm{wr}}\,l_{r}\,\cos\left(\theta _{r}\right)}{2\,R^2}-\frac{l_{r}\,m_{\mathrm{wl}}\,\cos\left(\theta _{r}\right)}{2}-\frac{l_{r}\,m_{\mathrm{wr}}\,\cos\left(\theta _{r}\right)}{2}.
$$

$$
\left(M\right)_{2,5}=I_{b}.
$$

$$
\left(M\right)_{3,1}=-l_{r}\,m_{\mathrm{wr}}\,\cos\left(\theta _{r}\right)-\frac{I_{\mathrm{wr}}\,l_{r}\,\cos\left(\theta _{r}\right)}{R^2}.
$$

$$
\left(M\right)_{3,2}=-R_{w}\,l_{r}\,m_{\mathrm{wr}}\,\cos\left(\theta _{r}\right)-\frac{I_{\mathrm{wr}}\,R_{w}\,l_{r}\,\cos\left(\theta _{r}\right)}{R^2}.
$$

$$
\left(M\right)_{3,3}=\frac{l_{r}\,\sin\left(\theta _{r}\right)\,\left(\frac{l_{l}\,m_{b}\,\sin\left(\theta _{l}\right)}{2}+l_{l}\,m_{l}\,\sin\left(\theta _{l}\right)\right)}{2}+\frac{l_{l}\,l_{r}\,m_{\mathrm{wr}}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2}+\frac{I_{\mathrm{wr}}\,l_{l}\,l_{r}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2\,R^2}.
$$

$$
\left(M\right)_{3,4}=I_{r}+\frac{l_{r}\,\sin\left(\theta _{r}\right)\,\left(\frac{l_{r}\,m_{b}\,\sin\left(\theta _{r}\right)}{2}+l_{r}\,m_{r}\,\sin\left(\theta _{r}\right)\right)}{2}+\frac{{l_{r}}^2\,m_{\mathrm{wr}}\,{\cos\left(\theta _{r}\right)}^2}{2}+\frac{I_{\mathrm{wr}}\,{l_{r}}^2\,{\cos\left(\theta _{r}\right)}^2}{2\,R^2}.
$$

$$
\left(M\right)_{4,1}=-l_{l}\,m_{\mathrm{wl}}\,\cos\left(\theta _{l}\right)-\frac{I_{\mathrm{wl}}\,l_{l}\,\cos\left(\theta _{l}\right)}{R^2}.
$$

$$
\left(M\right)_{4,2}=R_{w}\,l_{l}\,m_{\mathrm{wl}}\,\cos\left(\theta _{l}\right)+\frac{I_{\mathrm{wl}}\,R_{w}\,l_{l}\,\cos\left(\theta _{l}\right)}{R^2}.
$$

$$
\left(M\right)_{4,3}=I_{l}+\frac{l_{l}\,\sin\left(\theta _{l}\right)\,\left(\frac{l_{l}\,m_{b}\,\sin\left(\theta _{l}\right)}{2}+l_{l}\,m_{l}\,\sin\left(\theta _{l}\right)\right)}{2}+\frac{{l_{l}}^2\,m_{\mathrm{wl}}\,{\cos\left(\theta _{l}\right)}^2}{2}+\frac{I_{\mathrm{wl}}\,{l_{l}}^2\,{\cos\left(\theta _{l}\right)}^2}{2\,R^2}.
$$

$$
\left(M\right)_{4,4}=\frac{l_{l}\,\sin\left(\theta _{l}\right)\,\left(\frac{l_{r}\,m_{b}\,\sin\left(\theta _{r}\right)}{2}+l_{r}\,m_{r}\,\sin\left(\theta _{r}\right)\right)}{2}+\frac{l_{l}\,l_{r}\,m_{\mathrm{wl}}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2}+\frac{I_{\mathrm{wl}}\,l_{l}\,l_{r}\,\cos\left(\theta _{l}\right)\,\cos\left(\theta _{r}\right)}{2\,R^2}.
$$

$$
\left(M\right)_{5,1}=-\frac{R_{w}\,\left(2\,I_{\mathrm{wl}}-2\,I_{\mathrm{wr}}\right)}{2\,R^2}.
$$

$$
\left(M\right)_{5,2}=I_{\mathrm{yaw}}+\frac{R_{w}\,\left(2\,I_{\mathrm{wl}}\,R_{w}+2\,I_{\mathrm{wr}}\,R_{w}\right)}{2\,R^2}.
$$

$$
\left(M\right)_{5,3}=\frac{R_{w}\,\left(I_{\mathrm{wl}}\,l_{l}\,\cos\left(\theta _{l}\right)-I_{\mathrm{wr}}\,l_{l}\,\cos\left(\theta _{l}\right)\right)}{2\,R^2}.
$$

$$
\left(M\right)_{5,4}=\frac{R_{w}\,\left(I_{\mathrm{wl}}\,l_{r}\,\cos\left(\theta _{r}\right)-I_{\mathrm{wr}}\,l_{r}\,\cos\left(\theta _{r}\right)\right)}{2\,R^2}.
$$

### $B_{\mathrm{control}}$ 的非零元素

$$
\left(B_{\mathrm{control}}\right)_{1,3}=-\frac{1}{R}.
$$

$$
\left(B_{\mathrm{control}}\right)_{1,4}=-\frac{1}{R}.
$$

$$
\left(B_{\mathrm{control}}\right)_{2,1}=1.
$$

$$
\left(B_{\mathrm{control}}\right)_{2,2}=1.
$$

$$
\left(B_{\mathrm{control}}\right)_{3,1}=-1.
$$

$$
\left(B_{\mathrm{control}}\right)_{3,3}=\frac{l_{r}\,\cos\left(\theta _{r}\right)}{R}+1.
$$

$$
\left(B_{\mathrm{control}}\right)_{4,2}=-1.
$$

$$
\left(B_{\mathrm{control}}\right)_{4,4}=\frac{l_{l}\,\cos\left(\theta _{l}\right)}{R}+1.
$$

$$
\left(B_{\mathrm{control}}\right)_{5,3}=-\frac{R_{w}}{R}.
$$

$$
\left(B_{\mathrm{control}}\right)_{5,4}=\frac{R_{w}}{R}.
$$

### $g$ 的非零元素

$$
\left(g\right)_{1,1}=\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{l}\,\sin\left(\theta _{l}\right)}{2}-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{r}\,\sin\left(\theta _{l}\right)}{2}-\frac{{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{l}\,\sin\left(\theta _{r}\right)}{2}-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{\mathrm{wl}}\,\sin\left(\theta _{l}\right)}{2}-\frac{{\mathrm{dtheta}_{l}}^2\,l_{l}\,m_{\mathrm{wr}}\,\sin\left(\theta _{l}\right)}{2}+\frac{{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{r}\,\sin\left(\theta _{r}\right)}{2}-\frac{{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{\mathrm{wl}}\,\sin\left(\theta _{r}\right)}{2}-\frac{{\mathrm{dtheta}_{r}}^2\,l_{r}\,m_{\mathrm{wr}}\,\sin\left(\theta _{r}\right)}{2}-\frac{I_{\mathrm{wl}}\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,\sin\left(\theta _{l}\right)}{2\,R^2}-\frac{I_{\mathrm{wr}}\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,\sin\left(\theta _{l}\right)}{2\,R^2}-\frac{I_{\mathrm{wl}}\,{\mathrm{dtheta}_{r}}^2\,l_{r}\,\sin\left(\theta _{r}\right)}{2\,R^2}-\frac{I_{\mathrm{wr}}\,{\mathrm{dtheta}_{r}}^2\,l_{r}\,\sin\left(\theta _{r}\right)}{2\,R^2}.
$$

$$
\left(g\right)_{2,1}=g\,l_{b}\,m_{b}\,\sin\left(\theta _{b}+\theta _{\mathrm{b0}}\right).
$$

$$
\left(g\right)_{3,1}=-\frac{2\,R^2\,g\,l_{r}\,m_{b}\,\sin\left(\theta _{r}\right)-I_{\mathrm{wr}}\,{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,\sin\left(2\,\theta _{r}\right)+2\,R^2\,g\,l_{r}\,m_{l}\,\sin\left(\theta _{r}\right)+2\,R^2\,g\,l_{r}\,m_{r}\,\sin\left(\theta _{r}\right)+2\,R^2\,g\,l_{r}\,m_{\mathrm{wl}}\,\sin\left(\theta _{r}\right)-2\,R^2\,g\,l_{r}\,m_{\mathrm{wr}}\,\sin\left(\theta _{r}\right)+\frac{R^2\,{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{b}\,\sin\left(2\,\theta _{r}\right)}{2}+R^2\,{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{r}\,\sin\left(2\,\theta _{r}\right)-R^2\,{\mathrm{dtheta}_{r}}^2\,{l_{r}}^2\,m_{\mathrm{wr}}\,\sin\left(2\,\theta _{r}\right)-2\,I_{\mathrm{wr}}\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)-4\,R^2\,g\,l_{r,d}\,m_{r}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{\mathrm{r0}}\right)-4\,R^2\,g\,l_{r,d}\,m_{r}\,\cos\left(\theta _{\mathrm{r0}}\right)\,\sin\left(\theta _{r}\right)+R^2\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,m_{b}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)+2\,R^2\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,m_{l}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)-2\,R^2\,{\mathrm{dtheta}_{l}}^2\,l_{l}\,l_{r}\,m_{\mathrm{wr}}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)}{4\,R^2}.
$$

$$
\left(g\right)_{4,1}=-\frac{2\,R^2\,g\,l_{l}\,m_{b}\,\sin\left(\theta _{l}\right)-I_{\mathrm{wl}}\,{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,\sin\left(2\,\theta _{l}\right)+2\,R^2\,g\,l_{l}\,m_{l}\,\sin\left(\theta _{l}\right)+2\,R^2\,g\,l_{l}\,m_{r}\,\sin\left(\theta _{l}\right)-2\,R^2\,g\,l_{l}\,m_{\mathrm{wl}}\,\sin\left(\theta _{l}\right)+2\,R^2\,g\,l_{l}\,m_{\mathrm{wr}}\,\sin\left(\theta _{l}\right)+\frac{R^2\,{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{b}\,\sin\left(2\,\theta _{l}\right)}{2}+R^2\,{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{l}\,\sin\left(2\,\theta _{l}\right)-R^2\,{\mathrm{dtheta}_{l}}^2\,{l_{l}}^2\,m_{\mathrm{wl}}\,\sin\left(2\,\theta _{l}\right)-2\,I_{\mathrm{wl}}\,{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)-4\,R^2\,g\,l_{l,d}\,m_{l}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{\mathrm{l0}}\right)-4\,R^2\,g\,l_{l,d}\,m_{l}\,\cos\left(\theta _{\mathrm{l0}}\right)\,\sin\left(\theta _{l}\right)+R^2\,{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,m_{b}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)+2\,R^2\,{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,m_{r}\,\cos\left(\theta _{r}\right)\,\sin\left(\theta _{l}\right)-2\,R^2\,{\mathrm{dtheta}_{r}}^2\,l_{l}\,l_{r}\,m_{\mathrm{wl}}\,\cos\left(\theta _{l}\right)\,\sin\left(\theta _{r}\right)}{4\,R^2}.
$$

$$
\left(g\right)_{5,1}=\frac{R_{w}\,\left(l_{l}\,\sin\left(\theta _{l}\right)\,{\mathrm{dtheta}_{l}}^2+l_{r}\,\sin\left(\theta _{r}\right)\,{\mathrm{dtheta}_{r}}^2\right)\,\left(I_{\mathrm{wl}}-I_{\mathrm{wr}}\right)}{2\,R^2}.
$$

## 线性化边界

控制器矩阵 `A`、`B` 不是在零输入处单独导出的符号式，而是 `compute_lqr_and_export.m` 在每个静态平衡点 `(x_ref,u0)` 对完整非线性状态方程求数值雅可比。因此保留了 `B(q)u0` 的姿态耦合。
