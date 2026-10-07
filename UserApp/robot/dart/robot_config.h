/**
 * @file robot_config.h
 * @brief dart 发射架机器人全部硬件接线、机械参数与控制参数
 *
 * 电机布局(默认全部挂 hcan1):
 *   - yaw:    M2006 x1, 角度环串级 PID 控制发射架 yaw
 *   - belt:   M3508 x2, 水平对置驱动同步带挡块, 位置环串级 PID, 严格同步
 *   - screw:  M3508 x1, 扳机丝杆, 控制扳机整体位置(决定拉簧拉伸量=射力, 可调)
 *   - servo:  PWM 舵机 x1, 扳机的发射动作(卡位角扣住/释放角放开), 50Hz 占空比->角度
 *
 * 坐标约定:
 *   - 同步带逻辑坐标: 0 = 释放方向硬限位(堵转校准得到), 正方向 = 储能方向
 *   - 丝杆逻辑坐标:   0 = 丝杆零点(校准顶到的硬限位), 数值越大射力越大(方向按实机标定)
 *   - yaw 逻辑坐标:   0 = 开机位置
 *   - 舵机: 直接给角度, 映射到脉宽见 DART_SERVO_* 宏
 *   - 所有电机角度均为转子侧(编码器在减速箱前): 3508(P19) 输出轴角度 = 总角度/19,
 *     2006 输出轴角度 = 总角度/36
 *   - 同步带行程换算(带轮直径 33.62mm, 周长 ~105.62mm, 减速比 19):
 *       转子角度 = 行程(mm) / 105.62 * 360 * 19 ≈ 行程(mm) * 64.76
 *       行程(mm) = 转子角度 / 64.76
 *     满行程约 1m ≈ 64760° 转子侧; 丝杆行程由导程决定, 需另测, 勿用带轮公式
 *
 * @attention 所有行程/角度/力矩参数均为占位初值, 必须按实机机械结构标定;
 *            电机方向不对时只翻转对应 DART_XXX_REVERSE 宏;
 *            舵机换定时器/通道只改 DART_SERVO_PWM_TIM / DART_SERVO_PWM_CHANNEL 两行。
 * @warning 上电使能会自动执行校准(同步带与丝杆都会低速顶硬限位):
 *          若上次断电时发射平台仍被扳机锁住且拉簧处于储能状态, 丝杆校准移动扳机有意外释放风险,
 *          上电前务必确认已卸压(已发射完/平台未被卡住)。
 */
#pragma once

/* ================= 板型: 单板控制发射架 ================= */
#define ONE_BOARD
// #define CHASSIS_BOARD
// #define GIMBAL_BOARD

#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) || \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error "只能定义一个板型宏: ONE_BOARD / CHASSIS_BOARD / GIMBAL_BOARD"
#endif

/* ================= 硬件接线(CAN ID / 串口) ================= */
#define DART_YAW_CAN (&hcan1)     // yaw 电机总线
#define DART_BELT_CAN (&hcan1)    // 同步带电机总线
#define DART_SCREW_CAN (&hcan1)   // 扳机丝杆电机总线

#define DART_YAW_CAN_ID 1     // M2006 yaw
#define DART_BELT_L_CAN_ID 4  // M3508 同步带电机(左)
#define DART_BELT_R_CAN_ID 2  // M3508 同步带电机(右)
#define DART_SCREW_CAN_ID 3   // M3508 扳机丝杆

/* 遥控器串口: F407 用 huart3(DBUS), H7 用 huart5 */
#ifdef STM32H723XX
#define DART_RC_UART (&huart5)
#else
#define DART_RC_UART (&huart3)
#endif

/* ================= 电机方向 =================
 * 两个同步带电机镜像安装, 配置使得逻辑正方向均为储能方向;
 * 若实机转向与定义相反, 翻转对应宏即可。 */
#define DART_YAW_REVERSE MOTOR_DIRECTION_NORMAL
#define DART_BELT_L_REVERSE MOTOR_DIRECTION_REVERSE
#define DART_BELT_R_REVERSE MOTOR_DIRECTION_NORMAL  // 对置安装, 需反转
#define DART_SCREW_REVERSE MOTOR_DIRECTION_NORMAL

/* ================= 零位校准参数 =================
 * 上电使能后自动执行一次, **只做同步带**: 顶释放方向硬限位(两侧零点同步) -> 回撤到释放位置;
 * 全程严格限制电机力矩, 完成后才允许储能/发射。
 * 扳机丝杆**不参与自动校准**(dart_launcher.c 里丝杆校准代码已 #if 0 停用),
 * 丝杆零点直接取"开机位置", 因此 DART_SCREW_POS_* 是相对开机位置的角度。 */
#define DART_BELT_CALI_SPEED_DPS 1000.0f          // 同步带顶硬限位速度 (deg/s)
#define DART_BELT_CALI_DIRECTION (-1.0f)       // 同步带释放方向符号(零点取这一端); 实机相反时改 +1.0f
#define DART_BELT_CALI_MAX_OUT 1000.0f         // 同步带校准严格限流 (M3508 满量程 16384)
#define DART_BELT_CALI_INTEGRAL_LIMIT 1000.0f  // 同步带校准积分限幅(速度给定0时保持力矩只来自积分, 太小会锁不住往下漂)
#define DART_BELT_CALI_TIMEOUT_MS 75000        // 顶限位步超时 (ms): 最坏从储能底出发 49000°/1000°/s≈49s, ×1.5
#define DART_BELT_CALI_BACKOFF_DEG 1000.0f       // 顶到限位后回撤距离, 即释放位置(回缩位) (deg)
#define DART_BELT_CALI_BACKOFF_SPEED_DPS 1000.0f // 顶到限位后回撤限速 (deg/s, 顶死后必须慢速退开)
#define DART_CALI_STALL_SPEED_DPS 100.0f         // 堵转判定(同步带/丝杆共用): 速度阈值 (deg/s)
#define DART_CALI_STALL_MS 500                 // 堵转判定(同步带/丝杆共用): 持续时间 (ms)
#define DART_CALI_START_GRACE_MS 500           // 顶限位起步宽限: 进入步骤后先忽略堵转判定, 防起步误判

/* ================= 同步带(储能)参数 =================
 * 扳机锁定状态下是单向通道: 发射平台会滑过扳机继续被向后拉, 因此储能时同步带不会堵转,
 * 储能只按位置行程推进(拉到位 -> 限速复位到释放位置), 不依赖堵转判完成;
 * 仅当超过滑台行程等异常顶死时, 由堵转看门狗直接判故障停机(两侧同时转不动才算顶死)。
 * 双带控制采用**共模/差模解耦**(dart_launcher.c BeltSyncUpdate):
 *   - 共模环: 两侧平均位置跟目标, 两电机得到**完全相同**的速度给定 -> 出力均担(real_current 不再一大一小);
 *   - 差模环: 主动把两侧位置差收敛到基准 -> 不同步自动纠正, **只纠偏不停机**;
 *   各阶段限速通过共模环 MaxOut 实现, 最终不超过 DART_BELT_MAX_SPEED_DPS(电机级硬上限) */
#define DART_BELT_HOME_DEG DART_BELT_CALI_BACKOFF_DEG  // 释放位置(回缩位): 挡块退出发射平台活动范围
/* 储能行程: 零位(释放方向硬限位)起沿储能方向的电机轴角度。
 * 实测(调试档点动, 转子侧): 释放到底(硬限位)→储能到底 全程 ≈ 49000°(≈757mm);
 * 目标取 48000°(≈741mm), 留 ~1000°(≈15mm)余量 —— 目标顶死在机械末端会让到位容差判不过/
 * 触发储能堵转看门狗误报故障。余量按手感调整, 但务必 < 49000° */
#define DART_BELT_CHARGE_DEG 47000.0f
/* 位置到位容差: 30° ≈ 0.46 mm。校准时电流被严格限流, 位置环会停在摩擦平衡点
 * (实测稳定残差 ≈4 转子度), 容差若小于该残差就会永远判"没到位" -> 校准/储能超时 */
#define DART_BELT_POS_TOL_DEG 30.0f
/* "拉到底"容差(储能拉拽专用): 两侧同时顶死且离目标 < 此值 -> 视为拉到机械末端, **正常完成**;
 * 离目标还很远就顶死 -> 真卡滞/超行程 -> 故障。
 * (踩过: 目标恰好落在机械末端时, 到位容差永远判不到 -> 误判堵转故障 -> 断力矩被拉簧带飞) */
#define DART_BELT_END_TOL_DEG 5000.0f
#define DART_BELT_MAX_OUT 12000.0f                 // 正常工作电流限幅
#define DART_BELT_INTEGRAL_LIMIT 8000.0f          // 正常工作积分限幅(速度给定0时的保持力矩来源; f_Integral_Limit 会把 Iout 硬钳在这里)
#define DART_BELT_MAX_SPEED_DPS 30000.0f            // 速度给定硬上限 (共模/差模合成后再钳一次)
#define DART_BELT_HOME_SPEED_DPS 15000.0f           // 复位到释放位置/挡块回撤限速 (deg/s)
#define DART_BELT_CHARGE_SPEED_DPS 3600.0f         // 储能拉拽限速 (deg/s)
/* 共模/差模解耦参数(见 BeltSyncUpdate): 共模=原每电机角度环参数; 差模=纠偏强度
 * 差模输出是叠加到两电机速度给定上的 ±(Kp*偏差), 偏差大时收敛快, 但不宜大于共模限速 */
/* 共模/差模解耦参数(见 BeltSyncUpdate/BlendBeltIntegrals):
 * 共模 = 原每电机角度环参数; 差模输出是**力矩偏置**(经积分互融注入两台速度环 Iout, ±Ib),
 * 两台速度给定完全相同 -> P 项一致, 稳态电流差只剩 2*Ib。
 * @note 差模量纲已从 °/s 改为力矩偏置: 不要再走速度给定(经速度环Kp放大成几千的电流差) */
#define DART_BELT_POS_KP 40.0f          // 共模位置环 Kp
#define DART_BELT_POS_KI 0.0f           // 共模位置环 Ki
#define DART_BELT_SYNC_KP 120.0f        // 差模 Kp(力矩/°): 虚拟"弹簧"强度, 管偏差**值**
#define DART_BELT_SYNC_KI 6.0f          // 差模 Ki(力矩/(°·s))
#define DART_BELT_SYNC_KD 0.1f          // 差模 Kd(力矩/(°/s)): 虚拟"阻尼器", 管两侧**相对速度**(修"一快一慢")
#define DART_BELT_SYNC_KD_LPF_RC 0.02f  // 差模微分低通时间常数(s), 抑制编码器微分噪声
#define DART_BELT_SYNC_KI_LIMIT 500.0f  // 差模**积分**限幅(必须远小于输出上限!): 积分只做小幅稳态补偿。
                                        // 踩坑: 积分限幅=输出上限时, 瞬态把积分灌满卡死, 持续注入上千偏置 ->
                                        // 电流差几千、位置被拖着走、静止才慢慢回(积分反向才退出)
#define DART_BELT_SYNC_TORQUE_MAX 3000.0f  // 差模力矩偏置**输出**上限(M3508 满量程 16384): 需覆盖运动中的负载不对称,
                                        // 否则动态中纠不住偏差(偏差越跑越远, 停下来才收敛)
/* 软基准时间常数(s), 仅"力矩同步"模式生效: 差模基准以该时间常数跟随实际偏差。
 * 两侧负载路径不对称/运动学换算差会让偏差慢漂, 硬锁基准=持续较劲=电流差;
 * 软基准放行慢漂移, 只纠"变化快于 TAU"的窜动/振荡。
 * 均流优先调小(0.2~0.5); 窜动纠得慢就调大(1~2); 0 = 两种模式都用硬基准 */
#define DART_BELT_SYNC_BASELINE_TAU_S 0.5f
/* 负载感知模式切换(带方向差防抖), 判据 = 两台 real_current 绝对值之和:
 *   > LOAD_ON  -> 力矩同步(带载拉拽/保持): 均流优先, 软基准放行变形差
 *   < LOAD_OFF -> 位置同步(空载移动): 位置优先, 硬基准锁死偏差, 两侧走位齐
 *   ON/OFF 之间保持原模式。切换点按实机噪声调整, 两者差 15~25% 为宜 */
#define DART_BELT_SYNC_LOAD_ON 3000.0f
#define DART_BELT_SYNC_LOAD_OFF 2400.0f
#define DART_BELT_SYNC_LOAD_LPF_TAU_S 0.3f  // 切换判据低通时间常数(s): real_current 噪声大, 不滤会在阈值附近反复横跳
#define DART_BELT_SYNC_MODE_DWELL_MS 500    // 模式最短驻留(ms): 切换本身有扰动(清差模积分/基准跳变), 防连续翻转
/* 速度环积分互融系数(0~1): 两台带电机速度环积分(Iout)每周期互相靠拢。
 * 稳态力矩 u = P + I: 共模已保证 P 项相同, 若积分各自为政, 摩擦差异/起动不对称会被积分
 * "记住"并固化成力矩差(real_current 一大一小); 互融后积分收敛为共享的一份, 稳态出力均分。
 * 1.0 = 完全互融(推荐); 0 = 禁用; 若两侧较劲发热可适当调小 */
#define DART_BELT_INTEGRAL_BLEND 1.0f
#define DART_BELT_SYNC_WARN_DEG 200.0f             // 双电机位置偏差告警阈值 (deg≈3mm, 只提示不停机; 差模环会自动纠偏)
#define DART_CHARGE_STALL_SPEED_DPS 30.0f          // 储能堵转看门狗速度阈值 (deg/s, 超滑台行程会顶死)
#define DART_CHARGE_STALL_MS 300                  // 储能堵转看门狗持续时间 (ms)
#define DART_CHARGE_TIMEOUT_MS 60000               // 储能拉到位超时 (ms) -> 故障: 48000°/1200°/s=40s, ×1.5
#define DART_RETRACT_TIMEOUT_MS 60000              // 复位到释放位置超时 (ms) -> 故障: 同上量级

/* ================= 扳机丝杆(射力)参数 =================
 * 扳机整体装在丝杆上, 由 M3508 位置环控制;
 * 丝杆位置决定卡住发射平台时拉簧的拉伸量, 直接影响发射力量和速度(可调)。
 * 逻辑坐标零点 = 开机位置(不做自动校零), 数值越大射力越大(方向按实机标定)。
 * @note 每次上电的开机位置不同, 所以射力位置只在上电后相对可重复;
 *       需要跨上电绝对重复, 就恢复 dart_launcher.c 里的丝杆零位校准(#if 0 那段)。 */
#define DART_SCREW_POS_DEFAULT_DEG 30.0f   // 默认射力位置(相对开机位置, deg)
#define DART_SCREW_POS_MIN_DEG 10.0f       // 射力位置下限(相对开机位置, deg)
#define DART_SCREW_POS_MAX_DEG 90.0f       // 射力位置上限(相对开机位置, 须 < 丝杆可走行程)
#define DART_SCREW_POS_TOL_DEG 20.0f        // 位置到位容差 (deg)
#define DART_SCREW_MAX_OUT 6000.0f         // 正常工作电流限幅
#define DART_SCREW_INTEGRAL_LIMIT 2000.0f  // 正常工作积分限幅
#define DART_SCREW_MAX_SPEED_DPS 30000.0f    // 位置环输出限幅 = 最大速度
#define DART_SCREW_ADJ_DPS 30000.0f           // 右摇杆竖直满行程时射力调节速度 (deg/s)
#define DART_SCREW_SETTLE_TIMEOUT_MS 2000  // 储能前丝杆就位超时 (ms)

/* 扳机丝杆零位校准参数: **[已停用]** 自动校准不再执行丝杆校零, 这些宏当前无引用,
 * 仅保留供恢复 dart_launcher.c 里 #if 0 那段丝杆校准代码时使用 */
#define DART_SCREW_CALI_SPEED_DPS 3600.0f        // 顶硬限位速度 (deg/s)
#define DART_SCREW_CALI_DIRECTION (-1.0f)     // 顶限位方向(零点取这一端); 实机零点在另一端时改 +1.0f
#define DART_SCREW_CALI_MAX_OUT 3000.0f       // 校准严格限流 (M3508 满量程 16384)
#define DART_SCREW_CALI_INTEGRAL_LIMIT 800.0f // 校准积分限幅
#define DART_SCREW_CALI_TIMEOUT_MS 8000       // 顶限位/退开单步超时 (ms)
#define DART_SCREW_CALI_BACKOFF_DEG 5.0f      // 找到零点后至少退开距离 (deg)

/* ================= 扳机舵机(PWM 发射动作)参数 =================
 * 舵机通过 50Hz PWM 占空比控制角度: 脉宽线性映射到 [ANGLE_MIN, ANGLE_MAX]。
 * 换定时器/通道只改下面两行(所选定时器需在 CubeMX 配好对应通道的 PWM,
 * 建议预分频使计数 tick = 1us, 便于按脉宽微秒数计算)。 */
#define DART_SERVO_PWM_TIM (&htim1)             // ← 舵机 PWM 定时器(待定, 按实际接线改)
#define DART_SERVO_PWM_CHANNEL (TIM_CHANNEL_1)  // ← 舵机 PWM 通道
#define DART_SERVO_PWM_PERIOD_S (0.02f)         // 50Hz PWM 周期 (s)
#define DART_SERVO_PULSE_MIN_US 500.0f          // 0° 对应脉宽 (us, 实车 270° 舵机)
#define DART_SERVO_PULSE_MAX_US 2500.0f         // 行程上限对应脉宽 (us, 实车 270° 舵机)
#define DART_SERVO_ANGLE_MIN_DEG 0.0f           // 舵机机械角度下限 (deg)
#define DART_SERVO_ANGLE_MAX_DEG 270.0f         // 舵机机械角度上限 (deg, 实车 270° 舵机; 180° 舵机改 180)
#define DART_SERVO_CATCH_DEG 135.0f             // 卡位角: 扣住发射平台(待发/上电默认), 行程中点
#define DART_SERVO_RELEASE_DEG 140.0f             // 释放角: 放开发射平台(发射)
#define DART_SERVO_ADJ_DPS 60.0f                // 调试档舵机角度增量速度 (deg/s, 摇杆松手即停, 防误触发)
#define DART_SERVO_SETTLE_MS 300                // 舵机动作等待时间 (ms, 开环无反馈)
#define DART_FIRE_DWELL_MS 500                  // 发射时释放角保持时间 (ms)

/* ================= yaw 参数 ================= */
#define DART_YAW_SENSITIVITY_DPS 80000.0f   // 满杆 yaw 角速度 (deg/s)
#define DART_YAW_SOFT_LIMIT_DEG 200000.0f   // 相对开机位置的软限位 (deg)
#define DART_YAW_LEAD_LIMIT_DEG 100000.0f    // 目标角超前反馈的限幅, 防目标跑飞 (deg)
#define DART_YAW_MAX_OUT 10000.0f         // 电流限幅 (M2006/C610 满量程 10000)
#define DART_YAW_INTEGRAL_LIMIT 5500.0f  // 积分限幅

/* ================= 遥控器参数 ================= */
#define DART_RC_DEADZONE 20      // 摇杆/拨轮死区
#define DART_STICK_FULL 660      // 摇杆满行程
#define DART_TASK_DT_S 0.001f    // RobotTask 标称周期 (s)

/* DR16 操作映射(侧边拨轮已弃用: 实车拨轮损坏):
 *   分组原则: **左摇杆 = 同步带 + yaw, 右摇杆 = 扳机(丝杆 + 舵机)**, 拉同步带时手不会碰到扳机轴;
 *   同一执行器在正常/调试档保持同一摇杆轴, 切换档位不换手。
 *   右开关: 下 = 安全停机(全部电机停); 中 = 使能(首次使能自动校准); 上 = 使能+调试点动(不自动校准)
 *   左开关: 下 = 待机; 中(上升沿) = 储能命令(故障/未校准时为重新校准); 上(上升沿) = 发射命令
 *   左摇杆水平: yaw 增量式角度目标(两档一致)
 *   左摇杆竖直: 调试档 = 双同步带电机同速点动(上推=储能方向); 正常档不使用
 *   右摇杆竖直: 扳机丝杆, 两档同轴 —— 正常档 = 射力(丝杆位置)微调(仅 IDLE 生效, READY 扳机带载禁调),
 *                调试档 = 丝杆点动(上推=射力增大)
 *   右摇杆水平: 调试档 = 舵机角度增量(松手即停, 标定卡位角/释放角用); 正常档不使用
 *   调试流程: 上电右开关拨上档(不自动校准) -> 点动验方向/扫舵机角度 -> 拨回中档,
 *             动一下左开关(下->中)开始自动校准 -> 之后正常操作 */

/* ================= 调试点动参数 ================= */
/* 调试点动速度(转子侧), 均为满杆对应值, 实际随摇杆线性缩放.
 * 同步带与丝杆点动均采用"虚拟位置目标 + 位置环": 摇杆给速率, 目标积分推进;
 * 顶到机械限位/跟不上时暂停目标积分(DART_DEBUG_*_MAX_ERR_DEG), 反向打杆先重同步目标,
 * 松手后目标保持、位置环顶住不滑走. 同步带走共模/差模(两侧同一目标+自动纠偏), 丝杆单电机只走位置环. */
#define DART_DEBUG_BELT_MAX_SPEED_DPS 30000.0f  // 同步带点动满杆速率 (deg/s)
#define DART_DEBUG_SCREW_MAX_SPEED_DPS 20000.0f // 扳机丝杆点动满杆速度 (deg/s)
#define DART_DEBUG_SCREW_KP 30.0f // 丝杆点动跟踪P(速度前馈+P): 误差1°补30°/s; 纯位置环追杆上限=角度环Kp×误差上限, 故改前馈
#define DART_DEBUG_BELT_SYNC_SPEED_DPS 300.0f  // 摇杆回中后的两侧偏差收敛限速 (deg/s)
#define DART_DEBUG_BELT_SYNC_WARN_DEG 200.0f   // 点动偏差告警阈值(相对进入点动时的基准, 转子侧 ≈3mm)
#define DART_DEBUG_BELT_MAX_ERR_DEG 500.0f     // 同步带目标超前实测上限: 超过暂停积分(顶限位/跟不上不跑飞)
#define DART_DEBUG_SCREW_MAX_ERR_DEG 200.0f    // 丝杆目标超前实测上限: 超过暂停积分(顶到丝杆机械限位不再持续出力)
#define DART_DEBUG_SCREW_HOLD_SPEED_DPS 500.0f // 丝杆松手后回到虚拟目标的收敛限速 (deg/s)
#define DART_DEBUG_LOG_PERIOD_MS 500           // 调试档状态日志周期 (ms)

/* ================= 任务周期 ================= */
#define DART_SYNC_WARN_PERIOD_MS 1000  // 同步偏差告警的最小打印间隔

/**
 * @brief yaw 电机(M2006)配置: 角度环串级速度环
 */
#define DART_YAW_MOTOR_CONFIG(can_h, _id)                                      \
  {                                                                            \
      .motor_type = M2006,                                                     \
      .can_init_config =                                                       \
          {                                                                    \
              .can_handle = can_h,                                             \
              .tx_id = _id,                                                    \
          },                                                                   \
      .controller_setting_init_config =                                        \
          {                                                                    \
              .angle_feedback_source = MOTOR_FEED,                             \
              .speed_feedback_source = MOTOR_FEED,                             \
              .outer_loop_type = ANGLE_LOOP,                                   \
              .close_loop_type = SPEED_LOOP | ANGLE_LOOP,                      \
              .motor_reverse_flag = DART_YAW_REVERSE,                          \
              .feedback_reverse_flag = DART_YAW_REVERSE,                       \
          },                                                                   \
      .controller_param_init_config =                                          \
          {                                                                    \
              .speed_PID =                                                     \
                  {                                                            \
                      .Kp = 2.0f,                                              \
                      .Ki = 0.3f,                                              \
                      .Kd = 0.0f,                                              \
                      .MaxOut = DART_YAW_MAX_OUT,                              \
                      .IntegralLimit = DART_YAW_INTEGRAL_LIMIT,                \
                      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral, \
                  },                                                           \
              .angle_PID =                                                     \
                  {                                                            \
                      .Kp = 12.0f,                                             \
                      .Ki = 0.0f,                                              \
                      .Kd = 0.0f,                                              \
                      .MaxOut = DART_YAW_SENSITIVITY_DPS * 4.0f,               \
                      .DeadBand = 0.2f,                                        \
                  },                                                           \
          },                                                                   \
  }

/**
 * @brief 同步带电机(M3508)配置: 位置环串级速度环
 * @note 速度环**不启用** PID_ErrorHandle: 堵转判据改用速度阈值(DART_CALI_STALL_*),
 *       PID 堵转标志位置位后不手动清会一直挂着, 会把已过去的堵转带到后续阶段造成误故障
 */
#define DART_BELT_MOTOR_CONFIG(can_h, _id, _reverse)                                    \
  {                                                                                     \
      .motor_type = M3508,                                                              \
      .can_init_config =                                                                \
          {                                                                             \
              .can_handle = can_h,                                                      \
              .tx_id = _id,                                                             \
          },                                                                            \
      .controller_setting_init_config =                                                 \
          {                                                                             \
              .angle_feedback_source = MOTOR_FEED,                                      \
              .speed_feedback_source = MOTOR_FEED,                                      \
              .outer_loop_type = ANGLE_LOOP,                                            \
              .close_loop_type = SPEED_LOOP | ANGLE_LOOP,                               \
              .motor_reverse_flag = _reverse,                                           \
              .feedback_reverse_flag = _reverse,                                        \
          },                                                                            \
      .controller_param_init_config =                                                   \
          {                                                                             \
              .speed_PID =                                                              \
                  {                                                                     \
                      .Kp = 2.0f,                                                       \
                      .Ki = 0.8f,                                                       \
                      .Kd = 0.0f,                                                       \
                      .MaxOut = DART_BELT_MAX_OUT,                                      \
                      .IntegralLimit = DART_BELT_INTEGRAL_LIMIT,                        \
                      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,         \
                  },                                                                    \
              .angle_PID =                                                              \
                  {                                                                     \
                      .Kp = 20.0f,                                                      \
                      .Ki = 0.0f,                                                       \
                      .Kd = 0.0f,                                                       \
                      .MaxOut = DART_BELT_MAX_SPEED_DPS,                                \
                      .DeadBand = 0.5f,                                                 \
                  },                                                                    \
          },                                                                            \
  }

/**
 * @brief 扳机丝杆电机(M3508)配置: 位置环串级速度环
 * @note 速度环**不启用** PID_ErrorHandle: 丝杆校零(当前 #if 0 停用)恢复后用的是速度阈值判据
 */
#define DART_SCREW_MOTOR_CONFIG(can_h, _id, _reverse)                            \
  {                                                                              \
      .motor_type = M3508,                                                       \
      .can_init_config =                                                         \
          {                                                                      \
              .can_handle = can_h,                                               \
              .tx_id = _id,                                                      \
          },                                                                     \
      .controller_setting_init_config =                                          \
          {                                                                      \
              .angle_feedback_source = MOTOR_FEED,                               \
              .speed_feedback_source = MOTOR_FEED,                               \
              .outer_loop_type = ANGLE_LOOP,                                     \
              .close_loop_type = SPEED_LOOP | ANGLE_LOOP,                        \
              .motor_reverse_flag = _reverse,                                    \
              .feedback_reverse_flag = _reverse,                                 \
          },                                                                     \
      .controller_param_init_config =                                            \
          {                                                                      \
              .speed_PID =                                                       \
                  {                                                              \
                      .Kp = 2.0f,                                                \
                      .Ki = 0.5f,                                                \
                      .Kd = 0.0f,                                                \
                      .MaxOut = DART_SCREW_MAX_OUT,                              \
                      .IntegralLimit = DART_SCREW_INTEGRAL_LIMIT,                \
                      .Improve = PID_Integral_Limit | PID_Trapezoid_Intergral,   \
                  },                                                             \
              .angle_PID =                                                       \
                  {                                                              \
                      .Kp = 80.0f,                                               \
                      .Ki = 0.0f,                                                \
                      .Kd = 0.0f,                                                \
                      .MaxOut = DART_SCREW_MAX_SPEED_DPS,                        \
                      .DeadBand = 0.2f,                                          \
                  },                                                             \
          },                                                                     \
  }
