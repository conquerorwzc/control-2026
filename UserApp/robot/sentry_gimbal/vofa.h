#pragma once

// 摩擦轮散布调参的VOFA+回传，帧率跟随RobotTask（约1kHz），JustFloat共16通道：
//  0     friction_set            三轮共用速度目标（转子侧），°/s
//  1-3   各轮 speed_aps          轮速反馈，°/s；跌落深度、恢复时间、轮间对称
//  4-6   各轮指令电流            final_output/1000，A；看限幅饱和与振荡毛刺
//  7-9   各轮速度环积分          speed_PID.Iout/1000，A；验证IntegralLimit是否顶死
// 10-12  各轮 real_current       C610反馈转矩原始计数，手册未给量程，离线对4-6回归标定
// 13    拨盘 total_angle         rad；每+30°(0.524 rad)为喂出一发，离线切发
// 14    拨盘 velocity            rad/s；供弹节奏一致性
// 15    帧计数                   不连续即丢帧：DMA忙时模块层静默丢帧
// 1Mbps下68B/帧约占68%线速；链路不稳可降VOFA_BAUDRATE，帧率不变、丢帧由计数暴露。

#define VOFA_BAUDRATE 1000000U // 无线烧录器串口上限1Mbps；降档取921600/460800

void VofaInit(void); // 将huart1改到VOFA_BAUDRATE并注册VOFA串口；RobotInit末尾调用一次
void VofaTick(void); // 采样并发送一帧DMA JustFloat；RobotTask末尾调用
