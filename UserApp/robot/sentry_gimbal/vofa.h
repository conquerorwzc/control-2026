#pragma once

#if 0 // 保留原摩擦轮通道说明与配置。
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
#else

// 双 Pitch 基线采集：JustFloat，25 通道，最高 500 Hz，UART1 / 921600 / 8N1。
// 104 B/帧 * 500 帧/s * 10 bit/B = 520000 bit/s，占串口56.4%，余量43.6%。
// 原发射采集代码保留于 vofa.c 的 #if 0；不访问另一块板的拨弹盘。
//  0     时间戳 ms（实际采样间隔以此为准）
//  1     计划采样序号（DMA忙或无线丢帧可见跳号；任务延迟以时间戳判断）
//  2-3   左/右拨杆原值：1=上，3=中，2=下，0=无效；仅记录指令，不代表出弹
//  4     侧拨轮原值（连发选择可能锁存，分析时需保留切换之前的数据）
//  5-6   大 Pitch 角度环目标/编码器累计角度 rad
//  7-8   大 Pitch 速度环目标/编码器速度 rad/s
//  9-11  大 Pitch 总力矩指令/速度环 Iout/重力前馈 N*m
// 12-13  小 Pitch 指令角度/IMU Pitch deg
// 14     小 Pitch 编码器累计角度 rad
// 15-17  小 Pitch 速度环目标/IMU Gyro[0]/编码器速度 rad/s
// 18-20  小 Pitch 总力矩指令/速度环 Iout/重力前馈 N*m
// 21-23  IMU 安装修正及杆臂补偿后的 Accel[0..2] m/s^2（不是去重力加速度）
// 24     右摇杆竖直原值，验证目标是否因残余输入持续累加
// 力矩指令均为驱动 motor_reverse_flag 处理前的 PID 坐标；小 Pitch 发送时整体取反。
// 关闭时 PID 状态会清零，角度 Ref 不表示下一次使能目标；目标误差只统计使能稳定段。
// 本布局不采集电机反馈力矩；保留积分及前馈以区分控制输出来源。
#define VOFA_CHANNEL_NUM 25U
#define VOFA_BAUDRATE 921600U
#define VOFA_SAMPLE_PERIOD_MS 2U

void VofaInit(void);
void VofaTick(void);
#endif
