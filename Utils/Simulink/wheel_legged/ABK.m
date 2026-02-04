%L0的变化范围
L0s = 0.112 : 0.01 : 0.38;

Ks = zeros(2,6,length(L0s));%有length(L0s)个2x6的K矩阵

%循环length(L0s)次
for step = 1: length(L0s)
    %1.定义符号变量 
    syms theta theta_d theta_dd;       %theta1为theta的一阶导，theta2为theta的二阶导
    syms x x_d x_dd;                   %同上
    syms phi phi_d phi_dd;             %同上
    syms T Tp N P Nm Pm Nf t;

    %2.机器人结构参数
    R = 0.077;
    L = L0s(step) / 2;
    Lm = L0s(step) / 2;
    l = 0;
    mw = 0.7;
    mp = 1.3;
    M = 5;
    Iw = 0.5 * mw * R^2;
    Ip = mp*((Lm+L)^2+0.05^2)/12.0;
    Im = M*(0.38^2+0.3^2)/12.0;
    g = 9.8;

    %3.描述各个量之间的等量关系
    %3.1受力
    Nm = M*(x_dd + (L+Lm)*(cos(theta)*theta_dd - sin(theta)*theta_d^2) - l*(cos(phi)*phi_dd - sin(phi)*phi_d^2));
    Pm = M*g - M*((L+Lm)*(cos(theta)*theta_d^2 + sin(theta)*theta_dd) + l*(cos(phi)*phi_d^2 + sin(phi)*phi_dd));
    N = Nm + mp*(x_dd + L*(cos(theta)*theta_dd - sin(theta)*theta_d^2));
    P = Pm + mp*(g - L*(cos(theta)*theta_d^2 + sin(theta)*theta_dd));
    %3.2力矩
    equ1 = x_dd*(Iw/R+mw*R) + N*R - T;
    equ2 = Ip*theta_dd-Tp+T+(N*L+Nm*Lm)*cos(theta)-(P*L+Pm*Lm)*sin(theta);
    equ3 = Im*phi_dd-Pm*l*sin(phi)-Nm*l*cos(phi)-Tp;
    %3.3通过上述受力和力矩方程解出x2,theta2,phi2
    [x_dd,theta_dd,phi_dd]=solve(equ1,equ2,equ3,x_dd,theta_dd,phi_dd);

    %4雅可比矩阵
    %A矩阵描述X_dot矩阵与X矩阵的关系的
    JA = jacobian([theta_d;theta_dd;x_d;x_dd;phi_d;phi_dd],[theta theta_d x x_d phi phi_d]);
    %B矩阵描述X_dot矩阵与u矩阵的关系的
    JB = jacobian([theta_d;theta_dd;x_d;x_dd;phi_d;phi_dd],[T Tp]);
    %对A、B矩阵进行精细化处理(化为平衡状态)
    A = vpa(subs(JA,[theta theta_d x x_d phi phi_d],[0 0 0 0 0 0]));
    B = vpa(subs(JB,[theta theta_d x x_d phi phi_d],[0 0 0 0 0 0]));
    
    %5.离散化处理
    [G,H]=c2d(eval(A),eval(B),0.005);     %采样时间为0.005

    %6.求解Ks
    % 李识予参数
    % LQR_Q = diag([1000 100 500 100 5000 10]);
    % LQR_R = diag([1 0.25]);
    
    % LQR_Q = diag([1 1 500 100 5000 1]);
    % LQR_R = diag([5 1]);
    % 山海机甲参数？
    % LQR_Q = diag([10 10 2068 914 6394 10]);
    % LQR_R = diag([10 1]);
    LQR_Q = diag([10 10 500 100 5000 10]);
    LQR_R = diag([10 1]);
    % 能用 不太抖
    % LQR_Q = diag([1 1 500 100 5000 1]);
    % LQR_R = diag([10 1]);
    Ks(:,:,step) = dlqr(G,H,LQR_Q,LQR_R);

end

%对K的每个元素进行关于L0的拟合
LQR_K = sym('LOR_K',[2 6]);         %创建名为K的2x6矩阵
syms L0;

for x = 1:2
    for y = 1:6
        p = polyfit(L0s,reshape(Ks(x,y,:),1,length(L0s)),3);
        LQR_K(x,y) = p(1)*L0^3 + p(2)*L0^2 + p(3)*L0 + p(4);
    end
end

matlabFunction(LQR_K , 'File' , 'lqr_K');
vpa(subs(LQR_K,L0,0.2));