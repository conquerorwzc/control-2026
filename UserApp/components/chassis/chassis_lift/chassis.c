#include "chassis.h"

#include "arm_math.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "remote_control.h"
#include "user_lib.h"
/* Private macro -------------------------------------------------------------*/
#define LEFT 0
#define RIGHT 1
// 不用的腿设为1
#define DISABLE_LEG_REAR_LEFT 1   // 左后腿 (Leg 0)
#define DISABLE_LEG_REAR_RIGHT 1  // 右后腿 (Leg 1)
#define DISABLE_LEG_FRONT_LEFT 0  // 左前腿 (Leg 2)
#define DISABLE_LEG_FRONT_RIGHT 0 // 右前腿 (Leg 3)
// ==================== 【前腿（齿条）专属动力学调参区】 ====================
// 速度与柔顺度（梯形曲线参数）
// 决定了腿弹出的绝对速度
#define FRONT_TOTAL_TIME_SEC 1.0f // 目标：完成单次最大行程的总时间(秒)
// 决定了起步和刹车的柔和程度，越大越平滑，必须 < 总时间的一半
#define FRONT_ACCEL_TIME_SEC 0.4f // 加速/减速缓冲段的时间(秒)

// 力量与保护（动态电流限幅，大疆电机满载为 16384）
// 运动时的爆发力。给小了跑不到预定极速，给大了撞击时会切断螺丝]
#define FRONT_LEFT_MOVING_MAX_OUT  7000.0f  // 遇到阻力允许拉到 10000
#define FRONT_RIGHT_MOVING_MAX_OUT 7000.0f
// 静止时的保持力。只要能锁死齿条不掉下来即可，越小电机越不容易发烫
#define FRONT_LEFT_STOP_MAX_OUT    4000.0f   // 驻车力稍微给大点防掉
#define FRONT_RIGHT_STOP_MAX_OUT   4000.0f
// ==================== 【硬件基础信息】 ====================
#define CALI_TASK_FREQ 500.0f  // 标定任务运行频率 (Hz) ，在ostask里得知
#define GEAR_RATIO_REAR 19.0f  // 后腿减速比
#define GEAR_RATIO_FRONT 19.0f // 前腿减速比

// ==================== 【标定行为期望 (物理参数)】 ====================
// 速度期望 (输出轴物理角速度：度/秒)
// 后腿: 40mm/s * 90度/mm = 3600 度/秒
#define SPEED_RETRACT_REAR_DEG 3600.0f
#define SPEED_EXTEND_REAR_DEG 3600.0f
// 前腿: 40mm/s * 2.7度/mm = 108 度/秒
#define SPEED_RETRACT_FRONT_DEG 108.0f
#define SPEED_EXTEND_FRONT_DEG 108.0f

// 力量离合期望 (虚拟弹簧允许拉伸的最大输出轴度数，度数越大爆发的推力越大)
// 后腿 (丝杠): 机械优势极大，允许 2~5mm 误差即可爆发出满载推力
#define FORCE_ZERO_REAR_DEG 1000.0f // 收缩靠墙: 允许 2mm 误差 (2*90)
#define FORCE_MAX_REAR_DEG 450.0f   // 撑起车身: 允许 5mm 误差 (5*90)

// 前腿 (齿条): 机械优势小，需要允许 5~15mm 误差让 PID 积攒出足够大的电流
#define FORCE_ZERO_FRONT_DEG 135.0f // 收缩靠墙: 允许 5mm 误差 (5*2.7)
#define FORCE_MAX_FRONT_DEG 600.0f  // 撑起车身: 允许 15mm 误差 (15*2.7)

// 堵转静止判定期望 (允许的最大抖动速度：度/秒)
#define JITTER_TOLERANCE_REAR 15.0f
#define JITTER_TOLERANCE_FRONT 15.0f

// 判定时间窗口与安全系数
#define CALI_TIMEOUT_SEC 40.0f    // 全局防暴走超时保护时间 (秒)
#define ZERO_CHECK_SEC 1.5f       // 零点堵转持续判定时间 (秒)
#define MAX_CHECK_SEC 3.0f        // 伸展堵转持续判定时间 (秒)
#define MAX_CALI_SAFE_RATIO 0.99f // 机械限位保留系数

// 公式：度数 转化为 对应电机的 Ticks (1圈=360度=8192*减速比)
#define DEG_TO_TICKS(deg, ratio) ((deg) * (ratio) * 8192.0f / 360.0f)

// 时间转 Tick 循环次数
#define CALI_TIMEOUT_TICKS (uint32_t)(CALI_TIMEOUT_SEC * CALI_TASK_FREQ)
#define ZERO_CALI_CHECK_TICKS (uint32_t)(ZERO_CHECK_SEC * CALI_TASK_FREQ)
#define MAX_CALI_CHECK_TICKS (uint32_t)(MAX_CHECK_SEC * CALI_TASK_FREQ)

// 斜坡步长推导 (Step = 期望速度转成的Ticks / 频率)
#define ZERO_CALI_STEP_REAR (DEG_TO_TICKS(SPEED_RETRACT_REAR_DEG, GEAR_RATIO_REAR) / CALI_TASK_FREQ)
#define ZERO_CALI_STEP_FRONT (DEG_TO_TICKS(SPEED_RETRACT_FRONT_DEG, GEAR_RATIO_FRONT) / CALI_TASK_FREQ)
#define MAX_CALI_STEP_REAR (DEG_TO_TICKS(SPEED_EXTEND_REAR_DEG, GEAR_RATIO_REAR) / CALI_TASK_FREQ)
#define MAX_CALI_STEP_FRONT (DEG_TO_TICKS(SPEED_EXTEND_FRONT_DEG, GEAR_RATIO_FRONT) / CALI_TASK_FREQ)
// 滑动离合力矩推导 (转化为 Ticks 误差)
#define ZERO_CALI_SLIP_LIMIT_REAR DEG_TO_TICKS(FORCE_ZERO_REAR_DEG, GEAR_RATIO_REAR)
#define ZERO_CALI_SLIP_LIMIT_FRONT DEG_TO_TICKS(FORCE_ZERO_FRONT_DEG, GEAR_RATIO_FRONT)
#define MAX_CALI_SLIP_LIMIT_REAR DEG_TO_TICKS(FORCE_MAX_REAR_DEG, GEAR_RATIO_REAR)
#define MAX_CALI_SLIP_LIMIT_FRONT DEG_TO_TICKS(FORCE_MAX_FRONT_DEG, GEAR_RATIO_FRONT)
// 堵转静止容差推导 (允许抖动速度 * 判定时间 = 允许的 Ticks 波动范围)
#define ZERO_CALI_STOP_THRES_REAR DEG_TO_TICKS(JITTER_TOLERANCE_REAR *ZERO_CHECK_SEC, GEAR_RATIO_REAR)
#define ZERO_CALI_STOP_THRES_FRONT DEG_TO_TICKS(JITTER_TOLERANCE_FRONT *ZERO_CHECK_SEC, GEAR_RATIO_FRONT)
#define MAX_CALI_STOP_THRES_REAR DEG_TO_TICKS(JITTER_TOLERANCE_REAR *MAX_CHECK_SEC, GEAR_RATIO_REAR)
#define MAX_CALI_STOP_THRES_FRONT DEG_TO_TICKS(JITTER_TOLERANCE_FRONT *MAX_CHECK_SEC, GEAR_RATIO_FRONT)

/* Private variables ---------------------------------------------------------*/
static ChassisInstance *chassis;
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd; // 声明但不初始化
static Chassis_Param_s chassis_param;        // 声明为静态局部变量
static float chassis_vx, chassis_vy;         // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;     // 底盘速度解算后的临时输出,待进行限幅
static float lf_radius;
static float rf_radius;
static float lb_radius;
static float rb_radius;
static PIDInstance follow_pid;
static float k0, k1, k2, k3, k4, k5; // 中科大的功率模型
static float lift_speed_feedforward[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static float target_front_pos[2];
static float target_rear_pos[2];
static uint16_t cali_block_cnt[4] = {0, 0, 0, 0};
static uint16_t max_cali_block_cnt[4] = {0, 0, 0, 0};
static float cali_target_angle[4] = {0};
static float last_check_angle[4] = {0};
static uint8_t first_run = 1;
static uint32_t startup_grace_cnt = 0;
/* Private function prototypes -----------------------------------------------*/
static void MecanumCalculate();
static void PowerControl();
static void EstimateSpeed();
static void LimitChassisOutput();
static void ChassisCalibrationTask(void);
static void MaxExtensionCalibrationTask(uint8_t abort_flag);
static void Planner_Update(TrapezoidalPlanner_t *planner);
ChassisInstance *ChassisInit(Chassis_Init_Config_s *chassis_init_config);
static void LiftLeg_Init(LiftLeg_t *leg, float *ff_ch, uint8_t use_curve, float total_time, float acc_time,
                         float stroke, float move_out, float stop_out);
static void LiftLeg_SetTarget(LiftLeg_t *leg, float target);
static void LiftLeg_Execute(LiftLeg_t *leg);
void ChassisTask();
void Climb_FSM();
/* Private user code ---------------------------------------------------------*/
ChassisInstance *ChassisInit(Chassis_Init_Config_s *chassis_init_config)
{
    ChassisInstance *chassis_instance = (ChassisInstance *)zmalloc(sizeof(ChassisInstance));

    chassis_param = chassis_init_config->chassis_param; // 在运行时赋值

    float half_wheel_base = chassis_param.wheel_base / 2.0f;
    float half_track_width = chassis_param.track_width / 2.0f;
    float center_gimbal_offset_x = chassis_param.center_gimbal_offset_x;
    float center_gimbal_offset_y = chassis_param.center_gimbal_offset_y;
    k0 = chassis_param.power_param.k0;
    k1 = chassis_param.power_param.k1;
    k2 = chassis_param.power_param.k2;
    k3 = chassis_param.power_param.k3;
    k4 = chassis_param.power_param.k4;
    k5 = chassis_param.power_param.k5;

    lf_radius = sqrtf((half_track_width + center_gimbal_offset_x) * (half_track_width + center_gimbal_offset_x) +
                      (half_wheel_base - center_gimbal_offset_y) * (half_wheel_base - center_gimbal_offset_y)) *
                DEGREE_2_RAD;

    rf_radius = sqrtf((half_track_width - center_gimbal_offset_x) * (half_track_width - center_gimbal_offset_x) +
                      (half_wheel_base - center_gimbal_offset_y) * (half_wheel_base - center_gimbal_offset_y)) *
                DEGREE_2_RAD;

    lb_radius = sqrtf((half_track_width + center_gimbal_offset_x) * (half_track_width + center_gimbal_offset_x) +
                      (half_wheel_base + center_gimbal_offset_y) * (half_wheel_base + center_gimbal_offset_y)) *
                DEGREE_2_RAD;

    rb_radius = sqrtf((half_track_width - center_gimbal_offset_x) * (half_track_width - center_gimbal_offset_x) +
                      (half_wheel_base + center_gimbal_offset_y) * (half_wheel_base + center_gimbal_offset_y)) *
                DEGREE_2_RAD;

    PIDInit(&follow_pid, &chassis_init_config->follow_pid);

    for (int i = 0; i < 4; i++)
    {
        chassis_init_config->wheel_motor_config[i].controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
        chassis_init_config->wheel_motor_config[i].controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
        chassis_init_config->wheel_motor_config[i].controller_setting_init_config.outer_loop_type = SPEED_LOOP;
        chassis_init_config->wheel_motor_config[i].controller_setting_init_config.close_loop_type = SPEED_LOOP;
        chassis_instance->wheel_motor[i] = DJIMotorInit(&chassis_init_config->wheel_motor_config[i]);
    }
    for (int i = 0; i < 2; i++)
    {
        chassis_instance->front_legs[i].motor = DJIMotorInit(&chassis_init_config->lift_forward_motor_config[i]);
        chassis_instance->rear_legs[i].motor = DJIMotorInit(&chassis_init_config->lift_backward_motor_config[i]);

        // 绑定前馈通道给前腿底层电机
        chassis_instance->front_legs[i].motor->motor_settings.feedforward_flag = SPEED_FEEDFORWARD;
        chassis_instance->front_legs[i].motor->motor_controller.speed_feedforward_ptr = &lift_speed_feedforward[i + 2];
    }

    chassis = chassis_instance;

    // 初始化cmd
    chassis_ctrl_cmd = &chassis->chassis_ctrl_cmd;
    chassis_ctrl_cmd->backward_lift_in = chassis_param.backward_lift_in;
    chassis_ctrl_cmd->backward_lift_out = chassis_param.backward_lift_out;
    chassis_ctrl_cmd->forward_lift_out = chassis_param.forward_lift_out;
    chassis_ctrl_cmd->forward_lift_in = chassis_param.forward_lift_in;

    // 只让没有被屏蔽的腿去等待电流反馈上线
#if !DISABLE_LEG_REAR_LEFT
    while (chassis->rear_legs[0].motor->measure.real_current == 0)
        osDelay(10);
#endif
#if !DISABLE_LEG_REAR_RIGHT
    while (chassis->rear_legs[1].motor->measure.real_current == 0)
        osDelay(10);
#endif
#if !DISABLE_LEG_FRONT_LEFT
    while (chassis->front_legs[0].motor->measure.real_current == 0)
        osDelay(10);
#endif
#if !DISABLE_LEG_FRONT_RIGHT
    while (chassis->front_legs[1].motor->measure.real_current == 0)
        osDelay(10);
#endif

    chassis_ctrl_cmd->chassis_mode = CHASSIS_CALIBRATING;
    return chassis_instance;
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_CALIBRATING && RemoteControlIsOnline())
    {
        // 标定期间，轮电机断电，抬升电机上电
        for (int i = 0; i < 4; i++)
            DJIMotorStop(chassis->wheel_motor[i]);
        for (int i = 0; i < 2; i++)
        {
            DJIMotorEnable(chassis->front_legs[i].motor);
            DJIMotorEnable(chassis->rear_legs[i].motor);
        }

        // 执行非阻塞标定逻辑
        ChassisCalibrationTask();
        return; // 标定期间，直接 return，不执行下方正常的底盘解算和输出
    }

    // 只有在：处于爬楼模式 + 处于全伸出状态 + 最大行程没标定过 + 零点已经标定
    // 这四个条件同时满足时，才允许进入最大伸展标定
    // 🚨 拦截区：处理最大标定的进入与中途主动取消
       if (!chassis->cali_state.is_max_calibrated && chassis->cali_state.all_cali_done)
       {
           if (chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_BOTH_EXTEND)
           {
               // 正在请求标定：停止轮子，开启腿部
               for (int i = 0; i < 4; i++) DJIMotorStop(chassis->wheel_motor[i]);
               for (int i = 0; i < 2; i++) {
                   DJIMotorEnable(chassis->front_legs[i].motor);
                   DJIMotorEnable(chassis->rear_legs[i].motor);
               }
               MaxExtensionCalibrationTask(0); // 传入 0：正常运行标定
               return; // 标定期间阻断底层解算
           }
           else
           {
               // 用户中途切走了模式（主动取消标定）！
               MaxExtensionCalibrationTask(1); // 传入 1：触发任务内部重置清零！

               // 安全收腿保护：把腿安全地拉回物理零点，防止掉下来
               for (int i = 0; i < 2; i++) {
                   DJIMotorEnable(chassis->front_legs[i].motor);
                   DJIMotorEnable(chassis->rear_legs[i].motor);

                   // 温柔限流：避免瞬间全速收回砸碎限位，给个 8000 的安全电流
                   chassis->front_legs[i].motor->motor_controller.speed_PID.MaxOut = 8000.0f;
                   chassis->rear_legs[i].motor->motor_controller.speed_PID.MaxOut = 800.0f;

                   // 强制目标归零
                   DJIMotorSetPIDRef(chassis->front_legs[i].motor, chassis->cali_state.init_angle[2+i]);
                   DJIMotorSetPIDRef(chassis->rear_legs[i].motor, chassis->cali_state.init_angle[i]);
               }
               // 注意：这里没有 return，这意味着取消标定后，你可以正常用拨杆开着车到处跑！
           }
       }

    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_POWER_OFF)
    {
        // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        for (int i = 0; i < 4; i++)
            DJIMotorStop(chassis->wheel_motor[i]);
        for (int i = 0; i < 2; i++)
        {
            DJIMotorStop(chassis->front_legs[i].motor);
            DJIMotorStop(chassis->rear_legs[i].motor);
        }
    }
    else
    {
        // 正常工作
        for (int i = 0; i < 4; i++)
            DJIMotorEnable(chassis->wheel_motor[i]);
        for (int i = 0; i < 2; i++)
        {
            DJIMotorEnable(chassis->front_legs[i].motor);
            DJIMotorEnable(chassis->rear_legs[i].motor);
        }
    }
    // 根据控制模式设定旋转速度
    switch (chassis_ctrl_cmd->chassis_mode)
    {
    case CHASSIS_FOLLOW:
        chassis_ctrl_cmd->wz += PIDCalculate(&follow_pid, chassis_ctrl_cmd->offset_angle, 0);
        break;

    case CHASSIS_CLIMB_IDLE:
    case CHASSIS_CLIMB_BOTH_EXTEND:
        Climb_FSM();
        break;
    case CHASSIS_CLIMB_ALL_RETRACT:
    case CHASSIS_CLIMB_FRONT_RETRACT:
        chassis_ctrl_cmd->wz += PIDCalculate(&follow_pid, chassis_ctrl_cmd->offset_angle, 0);
        Climb_FSM();
        break;

    default:
        break;
    }
    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_ctrl_cmd->offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_ctrl_cmd->offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_ctrl_cmd->vx * cos_theta - chassis_ctrl_cmd->vy * sin_theta;
    chassis_vy = chassis_ctrl_cmd->vx * sin_theta + chassis_ctrl_cmd->vy * cos_theta;
    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    EstimateSpeed();

    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 功率控制与输出限幅
    LimitChassisOutput();
}

/**
 * @brief 梯形曲线更新器
 */
static void Planner_Update(TrapezoidalPlanner_t *planner)
{
    float remain_dist = fabsf(planner->target_pos - planner->current_ref);
    float decel_dist = (planner->current_vel * planner->current_vel) / (2.0f * planner->accel);

    if (remain_dist <= 5.0f)
    {
        planner->current_vel = 0.0f;
        planner->current_ref = planner->target_pos;
        planner->ff_speed = 0.0f;
        planner->is_moving = 0;
        return;
    }
    else if (remain_dist <= decel_dist)
    {
        planner->current_vel -= planner->accel;
        if (planner->current_vel < 1.0f)
            planner->current_vel = 1.0f;
    }
    else
    {
        if (planner->current_vel < planner->max_vel)
            planner->current_vel += planner->accel;
        else
            planner->current_vel = planner->max_vel;
    }

    if (planner->target_pos > planner->current_ref)
    {
        planner->current_ref += planner->current_vel;
        planner->ff_speed = planner->current_vel * CALI_TASK_FREQ;
    }
    else
    {
        planner->current_ref -= planner->current_vel;
        planner->ff_speed = -planner->current_vel * CALI_TASK_FREQ;
    }
    planner->is_moving = 1;
}
/**
 * @brief 对外开放：装配与初始化一条腿
 */
static void LiftLeg_Init(LiftLeg_t *leg, float *ff_ch, uint8_t use_curve, float total_time, float acc_time,
                         float stroke, float move_out, float stop_out)
{
    leg->ff_channel = ff_ch;
    leg->use_curve = use_curve;
    leg->moving_max_out = move_out;
    leg->stop_max_out = stop_out;

    float start_pos = leg->motor->measure.total_angle;
    leg->target_pos = start_pos;

    leg->planner.current_ref = start_pos;
    leg->planner.target_pos = start_pos;
    leg->planner.current_vel = 0.0f;
    leg->planner.ff_speed = 0.0f;
    leg->planner.is_moving = 0;

    if (use_curve)
    {
        // 防呆保护：防止加速时间设置的太离谱，自动修正
        if (acc_time >= total_time / 2.0f)
        {
            acc_time = total_time * 0.3f; // 强制把加速时间设为总时间的 30%
        }

        float v_max = fabsf(stroke) / (total_time - acc_time);

        if (v_max > 42000.0f)
            v_max = 42000.0f;
        leg->planner.max_vel = v_max / CALI_TASK_FREQ;
        leg->planner.accel = leg->planner.max_vel / (acc_time * CALI_TASK_FREQ);
    }
}

/**
 * @brief 设置目标坐标
 */
static void LiftLeg_SetTarget(LiftLeg_t *leg, float target)
{
    leg->target_pos = target;
    leg->planner.target_pos = target;
}
/**
 * @brief 目标坐标执行
 */
static void LiftLeg_Execute(LiftLeg_t *leg)
{
    if (leg->use_curve)
    {
        Planner_Update(&leg->planner);
        if (leg->ff_channel)
            *(leg->ff_channel) = leg->planner.ff_speed;

        leg->motor->motor_controller.speed_PID.MaxOut =
            leg->planner.is_moving ? leg->moving_max_out : leg->stop_max_out;
        DJIMotorSetPIDRef(leg->motor, leg->planner.current_ref);
    }
    else
    {
        if (leg->ff_channel)
            *(leg->ff_channel) = 0.0f;
        leg->motor->motor_controller.speed_PID.MaxOut = leg->stop_max_out;
        DJIMotorSetPIDRef(leg->motor, leg->target_pos);
    }
}

/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    vt_lf = -chassis_vx - chassis_vy - chassis_ctrl_cmd->wz * lf_radius;
    vt_rf = -chassis_vx + chassis_vy - chassis_ctrl_cmd->wz * rf_radius;
    vt_lb = chassis_vx - chassis_vy - chassis_ctrl_cmd->wz * lb_radius;
    vt_rb = chassis_vx + chassis_vy - chassis_ctrl_cmd->wz * rb_radius;
}

/**
 * @brief 功率模型
 * @todo 有待模块化,djimotor也得改改
 */
static void PowerControl()
{
    // 获取电机速度反馈,化成单位rad/s
    float motor_speed_fdb[4];
    for (int i = 0; i < 4; i++)
    {
        motor_speed_fdb[i] = (float)chassis->wheel_motor[i]->measure.speed_aps / 6.f;
    }

    // 获取当前电机参考电流，统一位单位为A
    float motor_current_list[4];
    for (int i = 0; i < 4; i++)
    {
        motor_current_list[i] = (float)chassis->wheel_motor[i]->motor_controller.final_output;
    }

    float initial_give_power[4] = {0.0f}; // 每个电机的初始估计功率
    float initial_total_power = 0.0f;     // 估计初始总功率

    // 计算每个电机的功率贡献
    for (int i = 0; i < 4; i++)
    {
        initial_give_power[i] =
            k0 + k1 * motor_current_list[i] / (16384.0f / 20.0f) + k2 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
            k3 * motor_current_list[i] / (16384.0f / 20.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
            k4 * motor_current_list[i] / (16384.0f / 20.0f) * motor_current_list[i] / (16384.0f / 20.0f) +
            k5 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f);

        // 只累加正向功率
        if (initial_give_power[i] > 0)
        {
            initial_total_power += initial_give_power[i];
        }
    }
    // 功率超限时进行动态调整
    if (initial_total_power > (float)chassis_ctrl_cmd->max_power)
    {
        float power_scale = (float)chassis_ctrl_cmd->max_power / initial_total_power; // 削减功率比例
        float scaled_give_power[4];
        // 计算缩放后的功率目标
        for (int i = 0; i < 4; i++)
        {
            scaled_give_power[i] = initial_give_power[i] * power_scale;
        }

        // 重新计算每个电机的电流参考值
        for (int i = 0; i < 4; i++)
        {
            // 二次方程系数计算，参数
            float a = k4 / (16384.0f / 20.0f) / (16384.0f / 20.0f);
            float b = k1 / (16384.0f / 20.0f) + k3 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) / (16384.0f / 20.0f);
            float c = k2 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
                      k5 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f) -
                      scaled_give_power[i] + k0;
            float discriminant = b * b - 4 * a * c; // 判别式
            if (discriminant >= 0)
            {
                float sqrt_disc = sqrtf(discriminant);
                float temp1 = (-b + sqrt_disc) / (2 * a);
                float temp2 = (-b - sqrt_disc) / (2 * a);

                // 选择最接近当前电流的解
                if (motor_current_list[i] > 0)
                {
                    motor_current_list[i] =
                        (fabsf(temp1 - motor_current_list[i]) < fabsf(temp2 - motor_current_list[i]))
                            ? fminf(16000.f, temp1)
                            : fminf(16000.f, temp2);
                }
                else
                {
                    motor_current_list[i] =
                        (fabsf(temp1 - motor_current_list[i]) < fabsf(temp2 - motor_current_list[i]))
                            ? fmaxf(-16000.f, temp1)
                            : fmaxf(-16000.f, temp2);
                }
            }
            else
            {
                // 无解时归零
                motor_current_list[i] = 0.0f;
            }
        }
    }
    for (int i = 0; i < 4; i++)
    {
        chassis->wheel_motor[i]->motor_controller.final_output = (int16_t)(motor_current_list[i]);
    }
}

/**
 * @brief 爬楼梯状态机
 */
void Climb_FSM()
{
    // 【1. 仅装配一次 4 条智能腿】
    static uint8_t is_legs_assembled = 0;
    if (!is_legs_assembled && chassis->cali_state.all_cali_done && chassis->cali_state.is_max_calibrated)
    {

        // 算出各自独立的极限行程 (物理真实行程 * 0.99)

        float stroke_front_l = fabsf(chassis->cali_state.max_angle[2] - chassis->cali_state.init_angle[2]) * MAX_CALI_SAFE_RATIO;
        float stroke_front_r = fabsf(chassis->cali_state.max_angle[3] - chassis->cali_state.init_angle[3]) * MAX_CALI_SAFE_RATIO;

        //左前腿装配：使用专属的 LEFT_MAX_OUT 宏
        LiftLeg_Init(&chassis->front_legs[LEFT], &lift_speed_feedforward[2], 1, FRONT_TOTAL_TIME_SEC,
                     FRONT_ACCEL_TIME_SEC, stroke_front_l, FRONT_LEFT_MOVING_MAX_OUT, FRONT_LEFT_STOP_MAX_OUT);

        //右前腿装配：使用专属的 RIGHT_MAX_OUT 宏
        LiftLeg_Init(&chassis->front_legs[RIGHT], &lift_speed_feedforward[3], 1, FRONT_TOTAL_TIME_SEC,
                     FRONT_ACCEL_TIME_SEC, stroke_front_r, FRONT_RIGHT_MOVING_MAX_OUT, FRONT_RIGHT_STOP_MAX_OUT);
        // 装配后腿：不用梯形曲线，stroke 随便传个 0 就行，依然靠纯位置环
        LiftLeg_Init(&chassis->rear_legs[LEFT], NULL, 0, 0, 0, 0, 1460.0f, 1460.0f);
        LiftLeg_Init(&chassis->rear_legs[RIGHT], NULL, 0, 0, 0, 0, 1200.0f, 1200.0f);

        is_legs_assembled = 1;
    }

    float front_l_target = chassis->cali_state.init_angle[2] +
                           (chassis->cali_state.max_angle[2] - chassis->cali_state.init_angle[2]) * MAX_CALI_SAFE_RATIO;
    float front_r_target = chassis->cali_state.init_angle[3] +
                           (chassis->cali_state.max_angle[3] - chassis->cali_state.init_angle[3]) * MAX_CALI_SAFE_RATIO;

    float rear_l_target = chassis->cali_state.init_angle[0] +
                          (chassis->cali_state.max_angle[0] - chassis->cali_state.init_angle[0]) * MAX_CALI_SAFE_RATIO;
    float rear_r_target = chassis->cali_state.init_angle[1] +
                          (chassis->cali_state.max_angle[1] - chassis->cali_state.init_angle[1]) * MAX_CALI_SAFE_RATIO;
    // ====================================================================

    switch (chassis_ctrl_cmd->chassis_mode)
    {
    case CHASSIS_CLIMB_IDLE:
    case CHASSIS_CLIMB_ALL_RETRACT:
        // 全员收缩：平滑回到各自的零点
        LiftLeg_SetTarget(&chassis->rear_legs[LEFT], chassis->cali_state.init_angle[0]);
        LiftLeg_SetTarget(&chassis->rear_legs[RIGHT], chassis->cali_state.init_angle[1]);
        LiftLeg_SetTarget(&chassis->front_legs[LEFT], chassis->cali_state.init_angle[2]);
        LiftLeg_SetTarget(&chassis->front_legs[RIGHT], chassis->cali_state.init_angle[3]);
        break;

    case CHASSIS_CLIMB_BOTH_EXTEND:
        // 全员出击：去往各自的独立安全极限坐标
        LiftLeg_SetTarget(&chassis->front_legs[LEFT], front_l_target);
        LiftLeg_SetTarget(&chassis->front_legs[RIGHT], front_r_target);
        LiftLeg_SetTarget(&chassis->rear_legs[LEFT], rear_l_target);
        LiftLeg_SetTarget(&chassis->rear_legs[RIGHT], rear_r_target);
        break;

    case CHASSIS_CLIMB_FRONT_RETRACT:
        // 越障姿态：前腿收回准备搭上台阶，后腿继续撑起车身
        LiftLeg_SetTarget(&chassis->rear_legs[LEFT], rear_l_target);
        LiftLeg_SetTarget(&chassis->rear_legs[RIGHT], rear_r_target);
        LiftLeg_SetTarget(&chassis->front_legs[LEFT], chassis->cali_state.init_angle[2]);
        LiftLeg_SetTarget(&chassis->front_legs[RIGHT], chassis->cali_state.init_angle[3]);
        break;

    default:
        break;
    }
}

/**
 * @brief 阶段一：底盘抬升零点标定任务 (收缩归零)
 * @note  引入滑动离合防震荡与高精度位移检测，各腿独立保存零点
 */
static void ChassisCalibrationTask(void)
{
    if (chassis->cali_state.all_cali_done)
        return;

    static uint32_t timeout_cnt = 0;
    timeout_cnt++;
    if (timeout_cnt > CALI_TIMEOUT_TICKS)
    {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        for (int i = 0; i < 2; i++)
        {
            DJIMotorSetRef(chassis->rear_legs[i].motor, 0);
            DJIMotorSetRef(chassis->front_legs[i].motor, 0);
        }
        LOGERROR("[Chassis] Calibration TIMEOUT! Partial data saved.");
        timeout_cnt = 0;
        first_run = 1;
        return;
    }

    if (first_run)
    {
        cali_target_angle[0] = chassis->rear_legs[0].motor->measure.total_angle;
        cali_target_angle[1] = chassis->rear_legs[1].motor->measure.total_angle;
        cali_target_angle[2] = chassis->front_legs[0].motor->measure.total_angle;
        cali_target_angle[3] = chassis->front_legs[1].motor->measure.total_angle;
        for (int i = 0; i < 4; i++)
        {
            last_check_angle[i] = cali_target_angle[i];
            cali_block_cnt[i] = 0;
        }

        // 👇 新增：极其优雅的免考逻辑，谁被屏蔽了，谁的标定就算做完成！
        chassis->cali_state.cali_done[0] = DISABLE_LEG_REAR_LEFT;
        chassis->cali_state.cali_done[1] = DISABLE_LEG_REAR_RIGHT;
        chassis->cali_state.cali_done[2] = DISABLE_LEG_FRONT_LEFT;
        chassis->cali_state.cali_done[3] = DISABLE_LEG_FRONT_RIGHT;

        first_run = 0;
    }
    if (startup_grace_cnt < ZERO_CALI_CHECK_TICKS)
        startup_grace_cnt++;

    for (int i = 0; i < 4; i++)
    {
        DJIMotorInstance *motor = (i < 2) ? chassis->rear_legs[i].motor : chassis->front_legs[i - 2].motor;

        if (!chassis->cali_state.cali_done[i])
        {
            float cali_step_size = (i < 2) ? ZERO_CALI_STEP_REAR : ZERO_CALI_STEP_FRONT;
            cali_target_angle[i] -= cali_step_size;

            float current_angle = motor->measure.total_angle;
            float slip_threshold = (i < 2) ? ZERO_CALI_SLIP_LIMIT_REAR : ZERO_CALI_SLIP_LIMIT_FRONT;
            if (cali_target_angle[i] < current_angle - slip_threshold)
            {
                cali_target_angle[i] = current_angle - slip_threshold;
            }

            DJIMotorSetPIDRef(motor, cali_target_angle[i]);

            if (startup_grace_cnt >= ZERO_CALI_CHECK_TICKS)
            {
                cali_block_cnt[i]++;
                if (cali_block_cnt[i] > ZERO_CALI_CHECK_TICKS)
                {
                    float check_threshold = (i < 2) ? ZERO_CALI_STOP_THRES_REAR : ZERO_CALI_STOP_THRES_FRONT;
                    float actual_diff = fabsf(current_angle - last_check_angle[i]);

                    // 加入零点电流双重判定，防止假零点
                    float actual_current = fabsf((float)motor->measure.real_current);

                    if (actual_diff < check_threshold && actual_current > 5000.0f)
                    {
                        chassis->cali_state.cali_done[i] = 1;
                        chassis->cali_state.init_angle[i] = current_angle;
                    }
                    last_check_angle[i] = current_angle;
                    cali_block_cnt[i] = 0;
                }
            }
        }
        else
        {
            // 继续保持拉力，别让腿掉下来
            DJIMotorSetPIDRef(motor, cali_target_angle[i]);
        }
    }

    chassis->cali_state.all_cali_done = chassis->cali_state.cali_done[0] && chassis->cali_state.cali_done[1] &&
                                        chassis->cali_state.cali_done[2] && chassis->cali_state.cali_done[3];
    if (chassis->cali_state.all_cali_done)
    {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        first_run = 1;
        timeout_cnt = 0;
        LOGINFO("[Chassis] Zero Calibration fully done!");
    }
}

/**
 * @brief 阶段二：最大伸展行程动态标定任务 (顶出)
 * @note  带有主动取消重置保护与双重堵转检测
 */
static void MaxExtensionCalibrationTask(uint8_t abort_flag)
{
    if (chassis->cali_state.is_max_calibrated)
        return;

    // 1. 把所有静态记忆变量集中到最顶端
    static uint32_t timeout_cnt = 0;
    static float cali_target_angle[4] = {0};
    static float last_check_angle[4] = {0};
    static uint8_t first_run = 1;
    static uint32_t startup_grace_cnt = 0;

    // 2. 🚨 新增：处理主函数发来的“记忆擦除”指令！(中途切遥控器取消标定)
    if (abort_flag)
    {
        timeout_cnt = 0;
        first_run = 1;         // 最重要的一步：重置起跑线！
        startup_grace_cnt = 0;
        for (int i = 0; i < 4; i++) {
            chassis->cali_state.max_cali_done[i] = 0;
            max_cali_block_cnt[i] = 0;
        }
        return; // 重置完毕，直接退出
    }

    // 3. 正常标定逻辑继续...
    timeout_cnt++;
    if (timeout_cnt > CALI_TIMEOUT_TICKS)
    {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        for (int i = 0; i < 2; i++)
        {
            DJIMotorSetRef(chassis->rear_legs[i].motor, 0);
            DJIMotorSetRef(chassis->front_legs[i].motor, 0);
        }
        LOGERROR("[Chassis] Max Ext Calibration TIMEOUT!");
        return;
    }

    if (first_run)
    {
        // 🚨 记录起跑线（绝对不能删），防止上电抽搐瞬移！
        cali_target_angle[0] = chassis->rear_legs[0].motor->measure.total_angle;
        cali_target_angle[1] = chassis->rear_legs[1].motor->measure.total_angle;
        cali_target_angle[2] = chassis->front_legs[0].motor->measure.total_angle;
        cali_target_angle[3] = chassis->front_legs[1].motor->measure.total_angle;
        for (int i = 0; i < 4; i++)
            last_check_angle[i] = cali_target_angle[i];

        // 👇 发最大行程的免考金牌！被屏蔽的腿直接视为标定完成
        chassis->cali_state.max_cali_done[0] = DISABLE_LEG_REAR_LEFT;
        chassis->cali_state.max_cali_done[1] = DISABLE_LEG_REAR_RIGHT;
        chassis->cali_state.max_cali_done[2] = DISABLE_LEG_FRONT_LEFT;
        chassis->cali_state.max_cali_done[3] = DISABLE_LEG_FRONT_RIGHT;

        first_run = 0;
    }

    if (startup_grace_cnt < MAX_CALI_CHECK_TICKS)
        startup_grace_cnt++;

    uint8_t current_all_done = 1;

    for (int i = 0; i < 4; i++)
    {
        DJIMotorInstance *motor = (i < 2) ? chassis->rear_legs[i].motor : chassis->front_legs[i - 2].motor;

        if (!chassis->cali_state.max_cali_done[i])
        {
            current_all_done = 0;
            float cali_step_size = (i < 2) ? MAX_CALI_STEP_REAR : MAX_CALI_STEP_FRONT;
            cali_target_angle[i] += cali_step_size;

            float current_angle = motor->measure.total_angle;
            float slip_threshold = (i < 2) ? MAX_CALI_SLIP_LIMIT_REAR : MAX_CALI_SLIP_LIMIT_FRONT;
            if (cali_target_angle[i] > current_angle + slip_threshold)
            {
                cali_target_angle[i] = current_angle + slip_threshold;
            }

            DJIMotorSetPIDRef(motor, cali_target_angle[i]);

            // 堵转位移 + 电流 双重结算
            if (startup_grace_cnt >= MAX_CALI_CHECK_TICKS)
            {
                max_cali_block_cnt[i]++;
                if (max_cali_block_cnt[i] > MAX_CALI_CHECK_TICKS)
                {
                    float check_threshold = (i < 2) ? MAX_CALI_STOP_THRES_REAR : MAX_CALI_STOP_THRES_FRONT;
                    float actual_diff = fabsf(current_angle - last_check_angle[i]);

                    // 获取电机当前的真实物理电流（绝对值）
                    float actual_current = fabsf((float)motor->measure.real_current);

                    LOGINFO("Leg[%d] Diff: %.1f, Curr: %.1f", i, actual_diff, actual_current);

                    // 不仅要“没怎么动”，而且必须是“憋足了劲（电流巨大）”！
                    if (actual_diff < check_threshold && actual_current > 6000.0f)
                    {
                        uint8_t allow_stop = 1; // 默认允许判定为撞墙停机

                        float current_stroke = fabsf(current_angle - chassis->cali_state.init_angle[i]);

                        if (i == 0 || i == 1)
                        { // 后腿
                            if (current_stroke < 250000.0f)
                                allow_stop = 0;
                        }
                        else if (i == 2 || i == 3)
                        { // 前腿
                            if (current_stroke < 2000.0f)
                                allow_stop = 0;
                        }

                        if (allow_stop)
                        {
                            chassis->cali_state.max_cali_done[i] = 1;
                            chassis->cali_state.max_angle[i] = current_angle;
                        }
                    }
                    last_check_angle[i] = current_angle;
                    max_cali_block_cnt[i] = 0;
                }
            }
        }
        else
        {
            // 继续维持虚拟弹簧的张力，让它保持 6000+ 的电流死死顶住墙壁，直到所有腿完成！
            DJIMotorSetPIDRef(motor, cali_target_angle[i]);
        }
    }

    if (current_all_done)
    {
        float rear_stroke_l = fabsf(chassis->cali_state.max_angle[0] - chassis->cali_state.init_angle[0]);
        float rear_stroke_r = fabsf(chassis->cali_state.max_angle[1] - chassis->cali_state.init_angle[1]);
        float front_stroke_l = fabsf(chassis->cali_state.max_angle[2] - chassis->cali_state.init_angle[2]);
        float front_stroke_r = fabsf(chassis->cali_state.max_angle[3] - chassis->cali_state.init_angle[3]);

        // 被屏蔽的腿，它的 max_angle 和 init_angle 是一样的，算出来行程是 0。
        // 用 fmaxf (取最大值)，系统就会自动抓取到那条真正跑了的腿的行程，当作梯形曲线的基准，方便总体观测
        chassis_ctrl_cmd->backward_lift_out = fmaxf(rear_stroke_l, rear_stroke_r);
        chassis_ctrl_cmd->forward_lift_out = fmaxf(front_stroke_l, front_stroke_r);

        chassis->cali_state.is_max_calibrated = 1;
        first_run = 1;
        LOGINFO("[Chassis] Max Ext Calibration done! Single leg baseline applied.");
    }
}

/**
 * @brief 预测电机功率并进行限制
 *
 */
static void LimitChassisOutput()
{
    DJIMotorSetPIDRef(chassis->wheel_motor[0], vt_lf);
    DJIMotorSetPIDRef(chassis->wheel_motor[1], vt_rf);
    DJIMotorSetPIDRef(chassis->wheel_motor[2], vt_lb);
    DJIMotorSetPIDRef(chassis->wheel_motor[3], vt_rb);

    if (chassis->cali_state.all_cali_done && chassis->cali_state.is_max_calibrated)
    {
        // 脱离了爬行模式，强行修改终极目标，安全收回所有腿

        // 💡 护城河逻辑升级：只要不属于这四种爬楼状态，立刻强制收缩护体！
        if (chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_IDLE &&
            chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_ALL_RETRACT &&
            chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_BOTH_EXTEND &&
            chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_FRONT_RETRACT)
        {
            LiftLeg_SetTarget(&chassis->front_legs[LEFT], chassis->cali_state.init_angle[2]);
            LiftLeg_SetTarget(&chassis->front_legs[RIGHT], chassis->cali_state.init_angle[3]);
            LiftLeg_SetTarget(&chassis->rear_legs[LEFT], chassis->cali_state.init_angle[0]);
            LiftLeg_SetTarget(&chassis->rear_legs[RIGHT], chassis->cali_state.init_angle[1]);
        }

        for (int i = 0; i < 2; i++)
        {
            LiftLeg_Execute(&chassis->front_legs[i]);
            LiftLeg_Execute(&chassis->rear_legs[i]);
        }
    }
    PowerControl();
}
/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
    // chassis_feedback_data.vx vy wz =
    // DJIMotor得改otherfeed
}
