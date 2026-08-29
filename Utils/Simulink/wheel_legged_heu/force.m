%通过关节电机测得的phi1和phi4以及对应加速度和速度等，来得到虚拟之后的量

%定义变量
syms phi1 phi2 phi3 phi4;
syms phi1_d phi4_d;
l1 = 0.17; 
l2 = 0.285; 
l3 = l2; 
l4 = l1; 
l5 = 0.16;
syms xc yc xd yd xb yb;

%描述变量之间对应的关系
xb = l1 * cos(phi1);
yb = l1 * sin(phi1);
xd = l5 + l4 * cos(phi4);
yd = l4 * sin(phi4);

A0 = 2 * l2 * (xd - xb);
B0 = 2 * l2 * (yd - yb);
C0 = l2^2 + (xd - xb)^2 + (yd - yb)^2 - l3^2;

phi2 = 2 * atan((B0 + sqrt(A0^2 + B0^2 - C0^2)) / (A0 + C0));

xc = xb + l2 * cos(phi2);
yc = yb + l2 * sin(phi2);

l0 = sqrt((xc - l5 / 2)^2 + yc^2);
phi0 = atan2(yc , xc - l5 / 2);

position = [l0 ; phi0];
matlabFunction(position , 'File' , 'leg_position');

%求雅各比矩阵
J11 = diff(l0 , phi1);
J12 = diff(l0 , phi4);
J21 = diff(phi0 , phi1);
J22 = diff(phi0 , phi4);
JacobianMatrix = [J11,J12;J21,J22];

%速度雅各比矩阵：关节->末端执行器
speed = JacobianMatrix * [phi1_d;phi4_d];
matlabFunction(speed , 'File' , 'leg_speed');

syms F Tp;
T = JacobianMatrix' * [F ; Tp];
matlabFunction(T , 'File' , 'leg_force');