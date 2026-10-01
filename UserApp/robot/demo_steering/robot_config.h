/**
 ******************************************************************************
 * @file    robot_config.h
 * @brief   四舵轮 demo 的全部可调参数集中在此文件
 *
 * 改装/换线时只需要改这个文件, 不需要动 robot.c 与底盘组件.
 ******************************************************************************
 */
#pragma once

#include "ins_task.h"
#include "robot.h"

// 编译warning,提醒开发者修改机器人参数
#ifndef ROBOT_CONFIG_PARAM_WARNING
#define ROBOT_CONFIG_PARAM_WARNING
#pragma message \
    "check if you have configured the parameters in robot_config.h, IF NOT, please refer to the comments AND DO IT, otherwise the robot will have FATAL ERRORS!!!"
#endif

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
#define CHASSIS_BOARD  // 底盘板: 只跑底盘相关逻辑
// #define ONE_BOARD   // 单板控制整车

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) || \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

/* ============================ 一、CAN 与电机 ID ============================ */

/**
 * @brief C620 / M3508 的报文标识符（DJI 标准协议，标识符由电机 ID 算出，不能自行指定）
 *
 *   - 控制帧: 0x200 覆盖电机 ID 1~4, 每台电机占 2 字节;
 *             0x1FF 覆盖电机 ID 5~8。这两帧由驱动按 ID 自动分组发送, 应用层不用配置。
 *   - 反馈帧: 0x200 + 电机 ID。ID = 4 → 0x204（驱动自动注册对应的硬件过滤器）。
 *
 * @attention 驱动里 CAN_Init_Config_s 的 tx_id 填的是**电机拨码 ID(1~8)**, 不是 CAN 帧标识符。
 *            把帧标识符直接填进 tx_id(例如 0x104)会让分组/发送槽位/接收过滤器全部算错,
 *            表现为"收不到反馈、帧也发不出去"。仓库内
 *            UserApp/robot/infantry_wheel_legged_demo/robot_config.h 有一处这样的误用, 不要照抄。
 */
#define DJI_WHEEL_CTRL_FRAME_LOW 0x200U   // 电机 ID 1~4 的控制帧
#define DJI_WHEEL_CTRL_FRAME_HIGH 0x1FFU  // 电机 ID 5~8 的控制帧
#define DJI_WHEEL_FEEDBACK_ID(id) (0x200U + (id))  // 反馈帧标识符

// 3508 轮电机全部挂在 CAN1; 若某个轮组改接到 CAN2, 把对应 config 宏的 handle 改成 (&hcan2) 即可.
#define DEMO_WHEEL_CAN (&hcan1)
// GM6020 舵电机挂在 CAN2, 阶段 A 不注册, 阶段 B 标定零偏后放开掩码.
#define DEMO_RUDDER_CAN (&hcan2)

// 电机 ID 数组, 下标顺序必须与底盘组件的 Chassis_Wheel_Index_e 一致: LF -> LB -> RB -> RF
// 注意: 这里的 ID 是 C620 / GM6020 电调上的拨码 ID(1~8), 不是数组下标.
// 单个轮组联调时, 若实际接的是别的轮组, 只需交换数组里的元素位置, 使能掩码同步调整即可.
#define DEMO_WHEEL_ID_LF 4
#define DEMO_WHEEL_ID_LB 1
#define DEMO_WHEEL_ID_RB 3  // 本次焊接完成的轮组: CAN1 上 ID = 4, 反馈报文 0x204, 控制报文 0x200
#define DEMO_WHEEL_ID_RF 2

#define DEMO_RUDDER_ID_LF 3
#define DEMO_RUDDER_ID_LB 2
#define DEMO_RUDDER_ID_RB 1
#define DEMO_RUDDER_ID_RF 4

/* ==================== 二、电机注册范围(与底盘组件版本一致) ==================== */

/**
 * @attention 本 demo 的底盘组件已同步为 dev 分支的 chassis_steering 版本(2026/1/1 重写版)。
 *
 * 该版本的 `ChassisInit()` 会【无条件注册 4 个 3508 + 4 个 6020】, 没有使能掩码的概念。
 * 这意味着:
 *
 *   - 上电即注册全部 8 个电机, 四个 GM6020 会立刻朝"零偏对应的绝对角度"闭环;
 *   - 组件已内置保护: `rudder_motor_offset` 中存在 0 时会拒绝使能舵电机
 *     (轮电机不受影响), 可用 `ChassisIsRudderReady()` 查询;
 *   - 组件还会在使能后的第一次任务里用【舵机实测角】播种舵角参考, 避免四个舵机
 *     一起朝参考初值 0 度冲过去(那等于一次满舵动作);
 *   - 超电与裁判系统都是可选的: 超电未配置时不注册 CAN 实例、不做状态机;
 *     没有裁判系统时功率上限退回下面 DEMO_CHASSIS_POWER_LIMIT 的设定值。
 *
 * 若要恢复"单轮组调试"能力, 需要重新给组件加掩码 —— 但那正是之前把解算改坏的原因之一,
 * 建议不要再动, 单轮组阶段用原版组件 + 只接一个轮组的硬件即可。
 */
// 相位 B 记录: 曾经用过的掩码写法(现已删除)
// #define DEMO_WHEEL_ENABLE_MASK  (1u << LF | 1u << LB | 1u << RB | 1u << RF)
// #define DEMO_RUDDER_ENABLE_MASK (1u << LF | 1u << LB | 1u << RB | 1u << RF)

/* ============================ 三、舵机零偏(标定在这里填) ============================ */

/**
 * @brief 四个 GM6020 的零位编码器偏移, 【下标顺序固定 LF LB RB RF】。
 *
 * 定义 = 舵轮指向【机械正前方】时的原始 ecd 读数。
 *
 * 标定步骤:
 *   1. 打开本文件的 DEMO_CALIB_MODE=1 -> 烧录(组件会拒绝使能舵电机, 手掰无阻力);
 *   2. 上电, 逐个用手把舵轮掰到指向车头正前方, 松手应停在原地;
 *   3. 按 LF LB RB RF 的顺序读 [dbg] 的 ecd= 四个值;
 *   4. 【按同样的顺序】填进下面这一行, DEMO_CALIB_MODE 置回 0 -> 重新烧录。
 *
 * @note 与两个 reverse flag 无关, 照抄读数即可, 不需要加减任何数。
 * @note 正前与正后的 ecd 正好差 4096(180°), 别标反。
 * @note 四个值里只要有一个是 0, 组件就会拒绝使能舵电机(保护), 上电会看到
 *       [demo] rudder NOT ready -> servo will stay disabled。
 */
// 标定完成后填这一行(示例值, 换成实测的四个 ecd):
//   {6091, 6831, 733, 5190}   =  LF, LB, RB, RF
#define DEMO_RUDDER_OFFSET_LF 6111
#define DEMO_RUDDER_OFFSET_LB 5217
#define DEMO_RUDDER_OFFSET_RB 724
#define DEMO_RUDDER_OFFSET_RF 5040

/* ============================ 四、调试开关 ============================ */

/**
 * @brief 零偏标定模式 —— 标定时【必须】打开它。
 *
 * 置 1 时, RobotInit() 会把四个零偏强制为 0, 组件因此**拒绝使能四个舵电机**
 * (但保留 CAN 反馈)。于是:
 *   - 舵机不会上电锁死, 也不会把轮子往某个目标角度拉;
 *   - 你可以放心用手把舵轮掰到机械正前方, 松手后它【停在原地】;
 *   - [dbg] 的 ecd= 仍会正常刷新(那是原始编码器值, 与零偏无关)。
 *
 * @attention 为什么需要它: 正常模式下若零偏已存在, 角度环会把舵轮闭环到
 *   "零偏对应的姿态"。此时用手掰轮子是在和电机较劲, 一松手就被拉走,
 *   读到的 ecd 是"电机想去哪"而不是"机械正前方在哪" —— 这样标出来的零偏
 *   会全错, 而且错得没有规律。
 *
 * @attention 标定完【务必置回 0】, 否则舵机永远不使能。
 */
#define DEMO_CALIB_MODE 0

/**
 * @brief 只验舵向, 不许车跑(第一次试舵向时打开)。
 *
 * 置 1 时: 轮电机的速度参考被强制为 0, 四个轮子【只转向、不滚动】。
 * 这样即使舵向有问题(跑飞/抽动), 车也不会窜出去, 可以安全地单独确认舵角方向。
 *
 * @note 轮电机仍然使能并做零速闭环, 会有一点抱死力矩, 属正常。
 * @note 确认舵向无误后【务必改回 0】, 否则车永远不会走。
 */
#define DEMO_RUDDER_ONLY_TEST 0

// 1 = 上电后给该轮电机一段缓升缓降的速度指令, 验证"通电是否真的会转".
// 速度环 Kp = 1 时, speed_PID 输出约等于目标速度, 因此速度目标 ≈ 电流指令, 属于近似开环,
// 仅用于台架通电验证, 调试时务必把轮子架空. 默认 0(只验证报文连通性, 电机保持零电流).
#define DEMO_SPIN_TEST 0
#define DEMO_SPIN_SPEED 1000.0f  // 速度目标(近似电流指令), 3508 满量程 16384
#define DEMO_SPIN_PERIOD_MS 1000 // 单个缓升缓降周期
#define DEMO_REPORT_PERIOD_MS 500 // 连通性自检打印周期(2Hz, 看得清)

/**
 * @brief 状态报告: 一行四个轮子, 只在"变了"或"有错"时才打印。
 *
 * 打开时终端是【安静】的 —— 静止且正常时不输出任何东西, 一旦你动摇杆或者出现
 * 故障才会打印, 所以不会像以前那样刷屏。
 *
 * 输出形如:
 *   [dbg] cmd(0,0,0) wheel=OK | LF st=0 cur=0 e=0 o=0
 *                               LB st=0 cur=0 e=0 o=0
 *                               RB st=0 cur=0 e=0 o=0
 *                               RF st=0 cur=0 e=0 o=0
 *
 * 列: st=舵角目标  cur=现在实际指向  e=跟随误差(cur-st, 应接近 0)  o=角度环输出
 */
#define DEMO_DEBUG_WHEELS 1
#define DEMO_DEBUG_PERIOD_MS 500 // 评估周期(实际打印只在内容变化时发生)

/* ======================= 三点五、虚拟云台(自旋平移) ======================= */

/**
 * @brief 虚拟云台总开关。
 *
 * 本车没有云台, 但板上有 IMU。把【上电瞬间的车头朝向】当作"虚拟云台朝向",
 * 之后车体怎么自转, 平移方向都锁死在这个朝向上 —— 于是可以一边自旋一边直行。
 *
 * 原理: IMU 的 Yaw 给出"底盘相对上电朝向转过了多少度", 把它写进
 * chassis_ctrl_cmd->offset_angle, 组件的 TransformCommandToChassisFrame()
 * 就会自动把世界系平移指令旋回当前底盘系。
 *
 * 置 0 = 整个功能编译掉, 手感完全回到"前进 = 车头方向"。
 */
#define DEMO_VIRTUAL_GIMBAL 1

/**
 * @brief IMU Yaw -> offset_angle 的符号, 取值只能是 +1.0f 或 -1.0f。
 *
 * 组件约定 +wz = 顺时针(俯视), 而 IMU 的 Yaw 由标准四元数公式解出
 * (QuaternionEKF.c 的 Yaw = atan2(...)) = 【逆时针为正】, 两者相反,
 * 所以默认 -1.0f。
 *
 * 【实测方法】静止时把右摇杆向右打到底(指令 +wz = 顺时针):
 *   [dbg] 里 off= 应该【增大】 -> 保持 -1.0f
 *   off= 减小                  -> 改成 +1.0f
 * 若改完自旋时车仍然画弧而不是直行, 就别再动这个符号了, 先确认
 * DEMO_SIGN_YAW 是否让"右打 = 顺时针"(见下面第五节)。
 */
#define DEMO_IMU_YAW_SIGN (-1.0f)

/**
 * @brief 上电 0 点的采样窗口长度(ms)。
 *
 * 不采用"延时后单点采样": INS_Init() 返回时 EKF 刚初始化, yaw 还在收敛,
 * 单点会被收敛残差污染。这里在窗口内【多次采样求平均】, 顺带吃掉采样噪声。
 * 默认 500ms, 在 200Hz~1kHz 的任务下能拿到几百个样本。
 *
 * @note 参考步兵的做法(infantry_wheel_legged_heu/robot.c:281): 用
 *       (YawTotalAngle - imu_boot_yaw) 这种【相对差】而不是绝对值, 这样
 *       yaw 的常值偏差会在差分里抵消掉。
 */
#define DEMO_IMU_SETTLE_MS 500

/**
 * @brief IMU 初始化配置。
 *
 * @attention 【不要】配置 offset_flag / GyroOffset。让 offset_flag 保持 0,
 *   INS_Init() 才会走【在线零偏标定】(INS_CalibrateGyroForDebug), 这也是步兵
 *   机器人的做法(gimbal_standard/gimbal.c 把结构体置零后传进去)。若改成 1,
 *   它就会直接采用 GyroOffset 里的离线值, 那份值未必对得上这颗 IMU。
 *
 * @attention 【上电操作规程】标定期间车必须【静止且水平】:
 *   - INS_CalibrateGyroForDebug() 会采样 5000 次求均值当零偏, 期间车在动,
 *     均值就不是零偏, 整个 yaw 基准会带上持续累积的偏置漂移;
 *   - 该函数内部有温度门控 while (温度不在 39~41℃), 冷机开机时会一直等到
 *     IMU 加热到位, 表现为 RobotInit() 卡住十几秒 —— 属正常, 别断电。
 */
static IMU_Init_Config_s imu_init_config = {
    .flag = 1, .scale = {1.0f, 1.0f, 1.0f}, .Yaw = 0.0f, .Pitch = 0.0f, .Roll = 0.0f};

/* ========================== 四、遥控器与底盘手感 ========================== */

// 底盘功率上限(W), 仅在【没有裁判系统且没有超电】时生效(台架调试即为此种情况)。
// 组件内部默认值也是 80W, 这里显式写出来是为了让整车联调时可以一处调整。
// @attention 若置 0, 组件的功率环会把四个轮电流削到 0, 表现为"轮子完全不动"。
#define DEMO_CHASSIS_POWER_LIMIT 80U

// 遥控器开关: 1 = 初始化遥控器并允许用拨杆驱动底盘.
// 只做台架报文连通性测试时置 0, 此时底盘锁在断电状态, 但电机仍会持续上报反馈.
#define DEMO_USE_REMOTE 1

// 遥控器串口: F407 用 huart3(DBUS), H7 用 huart5. 自研板或 VT13 请按实际接线改.
#ifdef STM32F407xx
#define DEMO_RC_UART (&huart3)
#elifdef STM32H723XX
#define DEMO_RC_UART (&huart5)
#endif

// 摇杆死区(原始值量程约 ±660), 防止松手时舵轮抖动
#define DEMO_RC_DEADBAND 30
#define DEMO_RC_STICK_RANGE 660.0f  // DBUS 摇杆原始量程, 用于归一化

/**
 * @brief 满杆对应的底盘指令.
 *
 * 【重要】vx / vy / wz 三者是**同一个量纲**(轮速量级的内部量, 不是 mm/s, 也不是度/秒)。
 * 组件里它们直接相加成轮速矢量:
 *     wheel_vector[i] = (vx, vy) + wz·(±√2/2, ±√2/2)
 *
 * 组合运动时单轮速度是**矢量叠加**。最坏工况(vx=vy=VMAX 且自转满杆)下:
 *     |v| = √( 2·VMAX² + 2·(WMAX·√2/2)² )
 * 上限是 MAX_WHEEL_SPEED = 40000。超出时:
 *   1) 轮速被 LimitWheelSpeed/LimitTranslationForSpin 削减, 且限幅只缩平移不缩自转,
 *      平移分量会被整个清零 → 表现为"平移和自转各转各的、轮子抽动";
 *   2) 超出电机速度环能力, 舵角环跟不上, ApplyRudderPriority 再把轮速压到 0。
 *
 * 下面取值的最坏工况余量:
 *     √(2×10000² + 2×8485²) ≈ 26200, 占上限 65%
 *
 * 参考: infantry_steering 用 wz = 42 * 摇杆值, 满杆约 ±27720。
 */
#define DEMO_RC_MAX_VX 30000.0f   // 满杆平移(前后)
#define DEMO_RC_MAX_VY 30000.0f   // 满杆平移(左右)
#define DEMO_RC_MAX_WZ 30000.0f   // 满杆自转: 自转分量 = 12000×0.707 ≈ 8485

/* ======================== 五、轴向符号总开关(调方向只改这里) ======================== */

/**
 * @brief 三个轴的极性开关, 取值只能是 +1.0f 或 -1.0f。
 *
 * 这三个符号是【操作层】的最终仲裁者: 不管前面哪一层的约定如何, 摇杆和车体动作的对应关系
 * 由它们决定。上板实测后对着改即可, 不需要动 chassis.c 的解算。
 *
 *   推左摇杆向上, 车向前   -> DEMO_SIGN_FWD 保持 +1; 向后则改 -1
 *   推左摇杆向右, 车向右移 -> DEMO_SIGN_LAT 保持 +1; 向左则改 -1
 *   推右摇杆向右, 车顺时针 -> DEMO_SIGN_YAW 保持 +1; 逆时针则改 -1
 *
 * @note 【st 读数的正确期望值】(st 以"舵轮指向机械正前方"为 0°, 指向右侧为 +90°)
 *         前进(vy+) -> 四轮全 0°        后退(vy-) -> 四轮全 ±180°
 *         右移(vx+) -> 四轮全 +90°      左移(vx-) -> 四轮全 -90°
 *         顺时针(wz+) -> LF +45  LB -45  RB -135  RF +135
 *       (自转那组是"从静止出发的短弧解", 四个角各自指向该处的顺时针切向;
 *        若舵机已经转起来, AngleToOptimalAngle 可能给等价的 ±180° 表示, 属正常。)
 *
 *       判读方法: 给某个轴一个指令, 看调试打印里的 st= 是否符合上表。
 *         st 对但车体动作错 -> 符号问题, 改对应的 DEMO_SIGN_*;
 *         st 本身就不对    -> 解算或零偏标定朝向问题, 不要用符号宏去凑。
 */
#define DEMO_SIGN_FWD (+1.0f)
#define DEMO_SIGN_LAT (+1.0f)
#define DEMO_SIGN_YAW (+1.0f)

/* ========================= 六、电机初始化配置模板 ========================= */

// 3508 轮电机: 速度环, 与 infantry_steering 保持一致的参数模板.
// 实测转向后若发现某个轮反转, 给对应 config 补 .motor_reverse_flag = MOTOR_DIRECTION_REVERSE.
#define WHEEL_MOTOR_CONFIG(handle, id)                                                                         \
  ((Motor_Init_Config_s){                                                                                      \
      .can_init_config =                                                                                       \
          {                                                                                                    \
              .can_handle = handle,                                                                            \
              .tx_id = id,                                                                                     \
          },                                                                                                   \
      .controller_param_init_config =                                                                          \
          {                                                                                                    \
              .speed_PID =                                                                                     \
                  {                                                                                            \
                      .Kp = 2,                                                                                 \
                      .Ki = 0,                                                                                 \
                      .Kd = 0,                                                                                 \
                      .IntegralLimit = 3000,                                                                   \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                      .MaxOut = 15000,                                                                         \
                  },                                                                                           \
              .current_PID =                                                                                   \
                  {                                                                                            \
                      .Kp = 0,                                                                                 \
                      .Ki = 0,                                                                                 \
                      .Kd = 0,                                                                                 \
                      .IntegralLimit = 3000,                                                                   \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                      .MaxOut = 15000,                                                                         \
                  },                                                                                           \
          },                                                                                                   \
      .controller_setting_init_config =                                                                        \
          {                                                                                                    \
              .angle_feedback_source = MOTOR_FEED,                                                             \
              .speed_feedback_source = MOTOR_FEED,                                                             \
              .outer_loop_type = SPEED_LOOP,                                                                   \
              .close_loop_type = SPEED_LOOP,                                                                   \
          },                                                                                                   \
      .motor_type = M3508,                                                                                     \
  })

// GM6020 舵电机: 角度-速度串级环. 底盘组件的 ConfigureRudderMotor() 也会强制一次, 这里保证单独使用时同样正确.
#define RUDDER_MOTOR_CONFIG(handle, id)                                                                        \
  ((Motor_Init_Config_s){                                                                                      \
      .can_init_config =                                                                                       \
          {                                                                                                    \
              .can_handle = handle,                                                                            \
              .tx_id = id,                                                                                     \
          },                                                                                                   \
      .controller_param_init_config =                                                                          \
          {                                                                                                    \
              .angle_PID =                                                                                     \
                  {                                                                                            \
                      .Kp = 35,                                                                                \
                      .Ki = 0,                                                                                 \
                      .Kd = 0.1f,                                                                              \
                      .IntegralLimit = 960,                                                                    \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                      .MaxOut = 1920,                                                                          \
                  },                                                                                           \
              .speed_PID =                                                                                     \
                  {                                                                                            \
                      .Kp = 9,                                                                                 \
                      .Ki = 25,                                                                                \
                      .Kd = 0,                                                                                 \
                      .IntegralLimit = 12500,                                                                  \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                      .MaxOut = 25000,                                                                         \
                  },                                                                                           \
              .current_PID =                                                                                   \
                  {                                                                                            \
                      .Kp = 0,                                                                                 \
                      .Ki = 0,                                                                                 \
                      .Kd = 0,                                                                                 \
                      .IntegralLimit = 3000,                                                                   \
                      .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement, \
                      .MaxOut = 15000,                                                                         \
                  },                                                                                           \
          },                                                                                                   \
      .controller_setting_init_config =                                                                        \
          {                                                                                                    \
              .angle_feedback_source = MOTOR_FEED,                                                             \
              .speed_feedback_source = MOTOR_FEED,                                                             \
              .outer_loop_type = ANGLE_LOOP,                                                                   \
              .close_loop_type = SPEED_LOOP | ANGLE_LOOP,                                                      \
              /* ----------------------------------------------------------------                              \
               * 两个标志管的是【不同】的环, 但都会改变该环的反馈极性:                                          \
               *                                                                                               \
               *   feedback_reverse_flag -> 反馈量(total_angle / speed_aps) 取负                                \
               *   motor_reverse_flag    -> 输出电流取负                                                        \
               *                                                                                               \
               * 【本车实测(2026-09, 唯一确认可用的一组)】                                                      \
               *     motor_reverse_flag    = MOTOR_DIRECTION_NORMAL                                             \
               *     feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL                                          \
               *   即【两个都 NORMAL】。                                                                         \
               *                                                                                               \
               *   实测过的其它组合(全部疯转/抽动, 不要再试):                                                    \
               *     (REVERSE, REVERSE)  坏                                                                     \
               *     (NORMAL , REVERSE)  坏                                                                     \
               *     (REVERSE, NORMAL )  坏                                                                     \
               *                                                                                               \
               * @attention 换舵机、换装配、改接线之后必须重新确认这一组值, 并【重新标定零偏】——                 \
               *            零偏是在某一组标志下测出来的, 换了标志就不作数。                                    \
               *                                                                                               \
               * @note 零偏 = 舵轮指向正前方时的【原始 ecd】。必须让舵机【失能】后再读,                          \
               *       否则读到的可能是被闭环拉住的位置而不是真正的机械正前方(见 README 标定章节)。              \
               *                                                                                               \
               * @note ForwardKinematicCal 读的是 angle_single_round(正数, 0~360), 它【不受】                   \
               *       feedback_reverse_flag 影响, 所以那里另外显式处理了方向, 两处不要互相套用。               \
               */                                                                                              \
              .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,                                                    \
              .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,                                              \
          },                                                                                                   \
      .motor_type = GM6020,                                                                                    \
  })

/* ============================ 五、底盘初始化配置 ============================ */

static Chassis_Init_Config_s chassis_init_config = {
    .chassis_param =
        {
            // 机械参数: 以下均为占位值, 整车装配完成后必须按实车测量更新, 单位 mm.
            .wheel_base = 350.0f,            // 纵向轴距(前进后退方向)
            .track_width = 300.0f,           // 横向轮距(左右平移方向)
            .center_gimbal_offset_x = 0.0f,  // 云台中心相对底盘中心的 x 轴偏移
            .center_gimbal_offset_y = 0.0f,  // 云台中心相对底盘中心的 y 轴偏移
            .wheel_radius = 60.0f,           // 轮子半径
            .wheel_reduction_ratio = 19.0f,  // 3508 减速比

            // 3508 功率模型参数(沿用 infantry_steering 的拟合结果)
            .power_param.k0 = 0.7441993412640775f,
            .power_param.k1 = 0.006444284468539646f,
            .power_param.k2 = 0.0001423857226262331f,
            .power_param.k3 = 0.015644430204543864f,
            .power_param.k4 = 0.1580143850678086f,
            .power_param.k5 = 2.896721772539512e-05f,

            /**
             * GM6020 舵机零位编码器偏移, 按 LF LB RB RF 顺序。
             *
             * 数值定义在文件顶部的 DEMO_RUDDER_OFFSET_* 宏里 —— 标定时只改那里,
             * 不用在几百行里找。详细标定步骤见那组宏上方的注释 / README。
             */
            /**
             * GM6020 舵机零位编码器偏移, 下标顺序固定为 LF LB RB RF。
             *
             * 用带下标的写法, 标定时【按 LF LB RB RF 的顺序】逐个填入实测 ecd 即可,
             * 不用在源码里找宏名。数值来源见文件顶部的 DEMO_RUDDER_OFFSET_* 注释。
             */
            .rudder_motor_offset =
                {
                    [LF] = DEMO_RUDDER_OFFSET_LF,
                    [LB] = DEMO_RUDDER_OFFSET_LB,
                    [RB] = DEMO_RUDDER_OFFSET_RB,
                    [RF] = DEMO_RUDDER_OFFSET_RF,
                },
        },

    // @note 本 demo 不接超级电容: super_cap_config 全部留空(can_handle == NULL),
    //       组件据此跳过超电注册与状态机, 功率上限改用 DEMO_CHASSIS_POWER_LIMIT。
    //       将来要接超电, 只要把下面 .super_cap_config 的 can_config 补全即可, 组件会自动启用状态机:
    //           .super_cap_config = { .can_config = { .can_handle = &hcan2,
    //                                                 .tx_id = 0x302,   // 超电默认接收 id
    //                                                 .rx_id = 0x301 } } // 超电默认发送 id
    //       (tx/rx 在别人看来是反的, 照抄 infantry_steering 的注释即可)
    //       注意超电会与舵电机共用 CAN2(@see DEMO_RUDDER_CAN), 接之前先确认总线负载。

    // 轮电机: 顺序 LF -> LB -> RB -> RF
    .wheel_motor_config[LF] = WHEEL_MOTOR_CONFIG(DEMO_WHEEL_CAN, DEMO_WHEEL_ID_LF),
    .wheel_motor_config[LB] = WHEEL_MOTOR_CONFIG(DEMO_WHEEL_CAN, DEMO_WHEEL_ID_LB),
    .wheel_motor_config[RB] = WHEEL_MOTOR_CONFIG(DEMO_WHEEL_CAN, DEMO_WHEEL_ID_RB),
    .wheel_motor_config[RF] = WHEEL_MOTOR_CONFIG(DEMO_WHEEL_CAN, DEMO_WHEEL_ID_RF),

    // 舵电机: 顺序 LF -> LB -> RB -> RF
    .rudder_motor_config[LF] = RUDDER_MOTOR_CONFIG(DEMO_RUDDER_CAN, DEMO_RUDDER_ID_LF),
    .rudder_motor_config[LB] = RUDDER_MOTOR_CONFIG(DEMO_RUDDER_CAN, DEMO_RUDDER_ID_LB),
    .rudder_motor_config[RB] = RUDDER_MOTOR_CONFIG(DEMO_RUDDER_CAN, DEMO_RUDDER_ID_RB),
    .rudder_motor_config[RF] = RUDDER_MOTOR_CONFIG(DEMO_RUDDER_CAN, DEMO_RUDDER_ID_RF),

    // 跟随 PID: 组件只在 CHASSIS_FOLLOW / CHASSIS_FOLLOW_DIAGONAL 模式下用它把
    // offset_angle 拉回目标值(带 5°/2° 滞回)。本 demo 用手动模式(CHASSIS_FREE),
    // 不会进入该分支, 但参数仍按可用值给出, 便于之后加"锁头"功能。
    .follow_pid =
        {
            .Kp = 300.0f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .IntegralLimit = 1000.0f,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .MaxOut = 40000.0f,
        },
};
