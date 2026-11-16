/**
 * @file robot_config.h
 * @brief dart 发射架全部硬件接线、机械参数与控制参数
 *
 * 执行器(默认全部挂 hcan1):
 *   - yaw:   M2006 x1, 角度环串级 PID(components/gimbal/gimbal_dart)
 *   - belt:  M3508 x2, 对置驱动同步带挡块, 共模/差模解耦 + 速度前馈(shoot_dart)
 *   - screw: M3508 x1, 扳机丝杆位置(决定拉簧拉伸量 = 射力, 可调)
 *   - servo: PWM 舵机 x1, 扳机发射动作(卡位角扣住 / 释放角放开), 50Hz 占空比 -> 角度
 *
 * 坐标约定(均为转子侧, 编码器在减速箱前):
 *   - 同步带: 0 = 释放方向硬限位(校准得到), 正方向 = 储能方向;
 *     行程换算: 带轮周长 ~105.62mm, 减速比 19 -> 转子角度 ≈ 行程(mm) * 64.76
 *   - 丝杆:   0 = 开机位置(不自动校零), 数值越大射力越大
 *   - yaw:    0 = 开机位置
 *
 * @warning 上电使能会自动执行同步带零位校准(低速顶释放方向硬限位)。若上次断电时发射平台仍被
 *          扳机锁住且拉簧处于储能状态, 上电前务必确认已卸压(已发射完/平台未被卡住)。
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
#define DART_YAW_CAN (&hcan1)    // yaw 电机总线
#define DART_BELT_CAN (&hcan1)   // 同步带电机总线
#define DART_SCREW_CAN (&hcan1)  // 扳机丝杆电机总线

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
 * 两个同步带电机镜像安装, 配置使逻辑正方向均为储能方向; 实机转向相反时翻转对应宏。 */
#define DART_YAW_REVERSE MOTOR_DIRECTION_NORMAL
#define DART_BELT_L_REVERSE MOTOR_DIRECTION_REVERSE
#define DART_BELT_R_REVERSE MOTOR_DIRECTION_NORMAL
#define DART_SCREW_REVERSE MOTOR_DIRECTION_NORMAL

/* ================= 零位校准(只做同步带) =================
 * 上电使能后自动执行: 双带以低力矩顶释放方向硬限位(堵转判据=速度阈值, 两侧都堵转才算到)
 * -> 各记两侧编码器零点 -> 回撤到释放位置。全程严格限流, 完成后才允许储能/发射。 */
#define DART_BELT_CALI_SPEED_DPS 1000.0f           // 顶硬限位速度 (deg/s)
#define DART_BELT_CALI_DIRECTION (-1.0f)           // 释放方向符号(零点取这一端); 实机相反改 +1.0f
#define DART_BELT_CALI_MAX_OUT 1000.0f             // 校准严格限流 (M3508 满量程 16384)
#define DART_BELT_CALI_INTEGRAL_LIMIT 1000.0f      // 校准积分限幅(给定 0 时的保持力矩只来自积分)
#define DART_BELT_CALI_TIMEOUT_MS 75000            // 顶限位步超时 (ms): 最坏 ~49000°/1000°/s≈49s, ×1.5
#define DART_BELT_CALI_BACKOFF_DEG 320.0f          // 顶到限位后回撤距离 = 释放位(挡块退离平台活动范围)
#define DART_BELT_CALI_BACKOFF_SPEED_DPS 1000.0f   // 回撤限速 (deg/s, 顶死后必须慢速退开)
#define DART_CALI_STALL_SPEED_DPS 100.0f           // 堵转判据: 实测速度阈值 (deg/s)
#define DART_CALI_STALL_MS 500                     // 堵转判据: 持续时间 (ms)
#define DART_CALI_START_GRACE_MS 500               // 起步宽限: 进入步骤后先忽略堵转判定

/* ================= 同步带(储能)行程/限速/力矩 =================
 * 扳机锁定态是单向通道: 平台滑过扳机后被继续后拉, 储能不会堵转 -> 按行程判完成;
 * 仅超行程等异常顶死时由堵转看门狗(两侧同时转不动)判故障。 */
#define DART_BELT_HOME_DEG DART_BELT_CALI_BACKOFF_DEG  // 释放位: 挡块退出发射平台活动范围
#define DART_BELT_CHARGE_DEG 35000.0f                  // 储能拉拽行程(相对零点, 正=储能方向);
                                                       // 实测全程 ≈49000°, 留 ~1000° 余量防顶死误报
#define DART_BELT_POS_TOL_DEG 30.0f                    // 到位容差(按两侧平均位置判) ≈0.46mm;
                                                       // 校准限流下位置环稳态残差 ≈4°, 容差必须更大
#define DART_BELT_END_TOL_DEG 5000.0f                  // "拉到底"容差: 两侧顶死且离目标 < 此值 = 拉到机械末端(正常完成)
#define DART_BELT_MAX_OUT 12000.0f                     // 正常工作电流限幅
#define DART_BELT_INTEGRAL_LIMIT 4000.0f               // 正常工作积分限幅(速度给定 0 时的保持力矩来源)
#define DART_BELT_MAX_SPEED_DPS 54000.0f               // 速度给定硬上限(共模/差模合成后再钳一次)
#define DART_BELT_HOME_SPEED_DPS 34000.0f              // 复位/回撤到释放位限速 (deg/s)
#define DART_BELT_CHARGE_SPEED_DPS 34000.0f            // 储能拉拽限速 (deg/s)

/* ---- 储能拉拽"推块咬住滑台"关键区限速 ----
 * 推块在 8000°(相对零点)附近咬住滑台: 从释放位高速通过这段时咬合瞬间接近撞击, 有损坏风险。
 * 所以该区间(±HALF_DEG)限速到 ENGAGE_SPEED, 并且进入区间前就按 DART_BELT_TRACK_ACC_DPS2
 * 的同一减速度曲线提前减速(不能等进了区间才刹, 那样已经撞上了); 咬合完成后恢复正常储能速度。
 * => 以后继续提高 DART_BELT_CHARGE_SPEED_DPS 只加快"咬合之后"的行程, 咬合过程始终安全。 */
#define DART_BELT_CHARGE_ENGAGE_POS_DEG 8000.0f     // 咬合位置 (deg, 相对零点)
#define DART_BELT_CHARGE_ENGAGE_HALF_DEG 500.0f     // 关键区半宽 (deg): 7500~8500 限速
#define DART_BELT_CHARGE_ENGAGE_SPEED_DPS 10000.0f  // 关键区速度上限 (deg/s, ≈154mm/s)
#define DART_BELT_CHARGE_ENGAGE_MARGIN_DEG 1000.0f  // 提前减速余量 (deg, ≈15mm): 补偿控制链滞后,
                                                   // 让实际速度到区间口前就降到上限; 实车到点还偏高就加大

/* ---- 回撤到释放位(靠近行程极限)的末段斜坡规划 ----
 * 释放位离行程硬限位只有 DART_BELT_CALI_BACKOFF_DEG(当前 300° ≈ 4.6mm): 从储能端高速回撤时,
 * 机构/控制链滞后会让滑台冲过释放位, 撞到同步带轮或硬限位。所以回撤末段按下面参数提前减速:
 *   距目标 APPROACH_DIST 内: 巡航限速降到 APPROACH_SPEED
 *   减速曲线用 APPROACH_DECEL(比 DART_BELT_TRACK_ACC_DPS2 小 = 刹车更早)
 *   最后 CREEP_DIST 内: 以 CREEP_SPEED 低速爬行到位
 * 只影响"回撤到释放位"这一段(校准回撤/储能回撤/就绪保持/故障保持/发射期间保持),
 * 不影响储能拉拽与点动。嫌回撤太慢就加大 APPROACH_SPEED / APPROACH_DECEL, 但必须保证
 * 末段速度足够低(CREEP_SPEED ≤ ~1000°/s), 否则又会冲过头。 */
#define DART_BELT_HOME_APPROACH_DIST_DEG 12000.0f   // 进入回撤末段的距离 (deg, ≈185mm)
#define DART_BELT_HOME_APPROACH_SPEED_DPS 12000.0f   // 回撤末段巡航限速 (deg/s, ≈124mm/s)
#define DART_BELT_HOME_APPROACH_DECEL_DPS2 800000.0f // 回撤末段减速曲线用的减速度 (deg/s², ≈185mm/s²)
#define DART_BELT_HOME_CREEP_DIST_DEG 700.0f        // 末段爬行距离 (deg, ≈6.2mm)
#define DART_BELT_HOME_CREEP_SPEED_DPS 250.0f      // 末段爬行速度 (deg/s, ≈12mm/s)
#define DART_BELT_HOME_CREEP_TAPER_DEG 100.0f      // 爬行速度随剩余距离线性收敛的参考距离 (deg, ≈1.5mm):
                                                   // 离目标越近速度越低, 到点速度≈0, 不会在释放位附近来回过冲

/* ================= 同步带共模/差模控制 =================
 * 两根带并联驱动同一滑块 -> 编码器位置差 θ0−θ1 正比于两侧皮带**张力差**
 * (1° 转子 ≈ 0.015mm 皮带行程差), 因此"位置一致"与"拉力一致"是同一个量:
 *   目标是**把位置差保持在标定基准 diff_ref 上**, 而不是让它->0。
 * - 差模环: 位置差低通+死区后 PD+I -> 输出电流偏置 u(机0 −u / 机1 +u), 经电流前馈叠加在
 *   速度环输出之后(不写速度环积分, 不会被积分限幅单边裁掉); 只纠偏不停机。
 * - 共模环: v = 速度前馈 + 位置修正 P(只补小误差), 两台同一速度给定 -> 出力均担。
 *   点动前馈 = 摇杆速率; 自动动作前馈 = 梯形规划(加速度限幅 + 末端减速曲线)。
 * - 均流: 两台速度环积分向均值互融, 摩擦差异不被积分固化成力矩差。
 * @note 执行链实际是 PID 在 RobotTask(~1kHz) 算、CAN 力矩在 MotorControlTask(~500Hz) 更新,
 *       回路纯滞后 ms 级 -> 差模环带宽不能高, 以下增益是实车调出来的, 不要为"静止不抖"收紧。 */
#define DART_BELT_POS_KP 40.0f           // 共模位置修正 P (速度/(°))
#define DART_BELT_SYNC_KP 40.0f          // 差模 P (电流/°): 管位置差(张力差)偏差
#define DART_BELT_SYNC_KI 3.0f           // 差模 I (电流/(°·s)): 提供稳态张力差记忆
#define DART_BELT_SYNC_KI_LIMIT 2000.0f  // 差模积分限幅 (电流)
#define DART_BELT_SYNC_KD_VEL 1.0f       // 差模阻尼 (电流/(°/s)): 直接作用于两台实测速度差
#define DART_BELT_SYNC_TORQUE_MAX 6000.0f // 差模电流偏置总限幅 (电流)
#define DART_BELT_SYNC_DIFF_LPF_TAU_S 0.008f // 位置差低通时间常数 (s): 抑制两机 CAN 帧不同时到达的假误差
#define DART_BELT_SYNC_DEADBAND_DEG 5.0f    // 位置差死区 (转子度): 跨过减速箱回差/齿隙
#define DART_BELT_INTEGRAL_BLEND 0.05f      // 速度环积分互融系数(0~1/周期): 只做纯均值慢互融

/* ---- 均张力(出力均衡)慢外环 ----
 * 差模环只保证"位置差"在标定基准上; 机械不对称(皮带自由长度/摩擦/对中)时, 等张力对应的
 * 位置差 ≠ 标定基准, 差模环会一直维持一个张力差 -> 一侧电机电流很小、几乎不出力。
 * 本外环用两侧电流差(低通)慢调位置差目标: 实际目标 = diff_base + tension_trim,
 * 直到两侧出力接近一致。速率慢(秒级)且 trim 限幅, 不会和位置差环抢, 也不会让一侧皮带松掉。 */
#define DART_BELT_BALANCE_KP 0.01f               // 快速均流增益 (deg/电流单位): 电流差 1000 -> 等效误差 10°,
                                                 // 经差模环(40倍)毫秒级压电流差; 环路增益≈2*40*Kb=0.8(<1 不振荡)。
                                                 // 想更狠可试 0.02~0.03, 出现高频抖动就降回来
#define DART_BELT_BALANCE_LPF_TAU_S 0.02f        // 快速均流用的电流差低通 (s): 滤 PWM/反馈噪声, 别太大(否则不快)
#define DART_BELT_BALANCE_MAX_DEG 1000.0f        // 快速均流项对差模误差的最大贡献 (deg): 防大电流差瞬间顶死差模环
#define DART_BELT_TENSION_TRIM_KP 0.3f           // 慢外环修正速率 (deg/s)/电流单位: 电流差 1000 -> 300°/s
#define DART_BELT_TENSION_TRIM_LPF_TAU_S 0.1f    // 慢外环电流差低通时间常数 (s)
#define DART_BELT_TENSION_TRIM_DEADBAND 100.0f   // 电流差死区 (raw): 小于此值不修正
#define DART_BELT_TENSION_TRIM_LIMIT_DEG 2000.0f // trim 限幅 (deg, ≈31mm): dev 允许偏离标定基准的最大量
#define DART_BELT_SYNC_WARN_DEG 200.0f      // 两侧位置差告警阈值 (deg, 只提示不停机)
#define DART_BELT_SYNC_LOG_MIN_SPEED_DPS 1000.0f  // 同步状态日志的速度门限(°/s): 静止不刷屏
#define DART_BELT_TRACK_ACC_DPS2 200000.0f   // 自动动作前馈加速度限幅 (°/s², ≈772mm/s²):
                                            // 起步时间 = 限速/a: 储能 10000 -> 200ms, 归位 25000 -> 500ms
#define DART_BELT_TRACK_P_CORR_DPS 800.0f   // 自动动作位置修正 P 上限 (°/s)
#define DART_BELT_TRACK_FF_V0_DPS 200.0f    // 减速曲线近端线性化速度 (°/s): v=sqrt(2a|e|+v0²)−v0

/* ================= 储能序列: 完成判定/超时 =================
 * 储能完成有两条判据, 谁先满足就走谁(直接决定节拍, 拉到底后不要多等):
 *   ① 到位: 两侧平均误差 < DART_BELT_CHARGE_TOL_DEG(比校准回撤用的 POS_TOL 宽);
 *   ② 顶到机械末端(目标超出实际行程): 两侧速度都低于 DART_CHARGE_STALL_SPEED_DPS 并持续
 *      DART_CHARGE_STALL_MS 即确认"拉到底", 立即回撤(不用等校准用的 500ms 慢判据)。
 * 顶死确认后会清掉前馈规划状态, 回撤指令立即生效(否则前馈要从 +v 慢慢降到负值才反向)。 */
#define DART_BELT_CHARGE_TOL_DEG 150.0f     // 储能到位容差 (deg, ≈2.3mm): 平台已被扳机锁住, 略差一点不影响射力
#define DART_CHARGE_STALL_SPEED_DPS 150.0f  // 储能"拉到底"堵转速度阈值 (deg/s)
#define DART_CHARGE_STALL_MS 150            // 储能"拉到底"堵转确认时间 (ms): 越短回撤越早, 太短易误判
#define DART_CHARGE_TIMEOUT_MS 10000        // 储能拉到位超时 (ms) -> 故障
#define DART_RETRACT_TIMEOUT_MS 10000       // 回撤到释放位超时 (ms) -> 故障

/* ================= 扳机丝杆(射力) =================
 * 扳机整体装在丝杆上, 丝杆位置决定卡住平台时拉簧的拉伸量 -> 射力。
 * 零点 = 开机位置(不做自动校零), 因此射力位置只在上电后相对可重复。 */
#define DART_SCREW_POS_MIN_DEG 10.0f        // 射力位置下限 (deg, 相对开机位置)
#define DART_SCREW_POS_MAX_DEG 90.0f        // 射力位置上限 (deg, 须 < 丝杆可走行程)
#define DART_SCREW_POS_TOL_DEG 20.0f        // 到位容差 (deg)
#define DART_SCREW_MAX_OUT 6000.0f          // 正常工作电流限幅
#define DART_SCREW_INTEGRAL_LIMIT 2000.0f   // 正常工作积分限幅
#define DART_SCREW_CALI_MAX_OUT 3000.0f     // 校准完成前(同步带未校准)的限流
#define DART_SCREW_CALI_INTEGRAL_LIMIT 800.0f // 校准完成前的积分限幅
#define DART_SCREW_MAX_SPEED_DPS 30000.0f   // 序列内位置环输出限幅 = 最大速度
#define DART_SCREW_ADJ_DPS 30000.0f         // 正常档右摇杆竖直满杆射力微调速度 (deg/s)
#define DART_SCREW_SETTLE_TIMEOUT_MS 2000   // 储能前丝杆就位超时 (ms)

/* ================= 扳机舵机(PWM 发射动作) =================
 * 50Hz PWM, 角度线性映射到脉宽; 换定时器/通道只改下面两行(定时器需在 CubeMX 配好 PWM)。 */
#define DART_SERVO_PWM_TIM (&htim1)            // 舵机 PWM 定时器
#define DART_SERVO_PWM_CHANNEL (TIM_CHANNEL_1) // 舵机 PWM 通道
#define DART_SERVO_PWM_PERIOD_S (0.02f)        // 50Hz (s)
#define DART_SERVO_PULSE_MIN_US 500.0f         // 角度下限对应脉宽 (us)
#define DART_SERVO_PULSE_MAX_US 2500.0f        // 角度上限对应脉宽 (us)
#define DART_SERVO_ANGLE_MIN_DEG 0.0f          // 舵机机械角度下限 (deg)
#define DART_SERVO_ANGLE_MAX_DEG 270.0f        // 舵机机械角度上限 (deg, 实车 270° 舵机; 180° 舵机改 180)
#define DART_SERVO_CATCH_DEG 135.0f            // 卡位角: 扣住发射平台(待发/上电默认)
#define DART_SERVO_RELEASE_DEG 0.0f          // 释放角: 放开发射平台
#define DART_SERVO_ADJ_DPS 60.0f               // 调试档舵机角度增量速度 (deg/s)
#define DART_SERVO_SETTLE_MS 1000              // 舵机动作等待时间 (ms, 开环无反馈)
#define DART_FIRE_DWELL_MS 300                 // 发射时释放角额外保持时间 (ms)

/* ================= yaw(云台) =================
 * components/gimbal/gimbal_dart: 摇杆给角速度, 积分成角度目标后角度环跟踪 */
#define DART_YAW_SENSITIVITY_DPS 100000.0f  // 满杆 yaw 角速度 (deg/s)
#define DART_YAW_SOFT_LIMIT_DEG 400000.0f   // 相对开机位置的软限位 (deg)
#define DART_YAW_LEAD_LIMIT_DEG 100000.0f   // 目标角超前反馈的限幅, 防目标跑飞
#define DART_YAW_MAX_OUT 10000.0f           // 电流限幅 (M2006/C610 满量程 10000)
#define DART_YAW_INTEGRAL_LIMIT 5500.0f     // 积分限幅

/* ================= 遥控器 =================
 * DR16 操作映射(侧边拨轮已弃用):
 *   左摇杆水平 = yaw; 左摇杆竖直 = 调试档同步带点动(上推=储能方向)
 *   右摇杆竖直 = 扳机丝杆(正常档=射力微调, 调试档=点动); 右摇杆水平 = 调试档舵机角度增量
 *   右开关: 下=安全停机, 中=使能(首次自动校准), 上=使能+调试点动(不自动校准)
 *   左开关: 下=待机, 中(上升沿)=储能(故障/未校准=重新校准), 上(上升沿)=发射(须经中档, 两段式防误发) */
#define DART_RC_DEADZONE 20   // 摇杆死区
#define DART_STICK_FULL 660   // 摇杆满行程
#define DART_TASK_DT_S 0.001f // RobotTask 标称周期 (s)

/* ================= 调试档点动 =================
 * 摇杆给速率 -> 积分出虚拟位置目标(两侧共用), 共模给两侧同一速度给定(均载), 差模自动纠偏。
 * 顶到机械限位/跟不上时暂停目标积分, 反向打杆先重同步目标, 松手收回目标领跑量(松手即停)。 */
#define DART_DEBUG_BELT_MAX_SPEED_DPS DART_BELT_MAX_SPEED_DPS // 同步带点动满杆速率 = 电机速度上限
#define DART_DEBUG_SCREW_MAX_SPEED_DPS 50000.0f  // 丝杆点动满杆速度 (deg/s)
#define DART_DEBUG_SCREW_KP 30.0f                // 丝杆点动跟踪 P(速度前馈 + P): 误差 1° 补 30°/s
#define DART_DEBUG_BELT_SYNC_SPEED_DPS 300.0f    // 摇杆回中后的两侧偏差收敛限速 (deg/s)
#define DART_DEBUG_BELT_SYNC_WARN_DEG 200.0f     // 点动偏差告警阈值(相对进入点动时的基准)
#define DART_DEBUG_BELT_MAX_ERR_DEG 500.0f       // 同步带目标超前实测上限: 超过暂停积分(不跑飞)
#define DART_DEBUG_SCREW_MAX_ERR_DEG 200.0f      // 丝杆目标超前实测上限: 超过暂停积分
#define DART_DEBUG_SCREW_HOLD_SPEED_DPS 500.0f   // 丝杆松手后收敛到虚拟目标的限速 (deg/s)
#define DART_DEBUG_LOG_PERIOD_MS 500             // 调试档状态日志周期 (ms)

/* ================= 日志周期 ================= */
#define DART_SYNC_WARN_PERIOD_MS 1000  // 同步偏差/同步状态日志的最小打印间隔 (ms)

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
 * @brief 同步带电机(M3508)配置: 速度环 + 电流前馈(差模偏置注入)
 * @param _ff_ptr 差模电流偏置指针(叠加在速度环输出之后, 不写速度环积分)
 * @note 速度环不启用 PID_ErrorHandle 堵转标志位(置位后需手动清, 会把已过去的堵转带进后续阶段)
 */
#define DART_BELT_MOTOR_CONFIG(can_h, _id, _reverse, _ff_ptr)                           \
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
              .feedforward_flag = CURRENT_FEEDFORWARD,                                  \
          },                                                                            \
      .controller_param_init_config =                                                   \
          {                                                                             \
              .current_feedforward_ptr = (_ff_ptr),                                     \
              .speed_PID =                                                              \
                  {                                                                     \
                      .Kp = 1.0f,                                                       \
                      .Ki = 0.5f,                                                       \
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
