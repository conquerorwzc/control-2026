#include "chassis.h"

#include "arm_math.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "remote_control.h"
#include "user_lib.h"

/* Private macro -------------------------------------------------------------*/
#define LEFT 0
#define RIGHT 1

// ==================== 【四腿独立免考/屏蔽开关】 ====================
// 设为 1 代表屏蔽该腿（不参与标定和运动），设为 0 代表正常工作
// ⚠️ 注意：正常比赛时，必须全部为 0！
#define DISABLE_LEG_REAR_LEFT 0   // 左后腿 (Leg 0)
#define DISABLE_LEG_REAR_RIGHT 0  // 右后腿 (Leg 1)
#define DISABLE_LEG_FRONT_LEFT 0  // 左前腿 (Leg 2)
#define DISABLE_LEG_FRONT_RIGHT 0 // 右前腿 (Leg 3)

// ==================== 【前腿（齿条）专属动力学调参区】 ====================
#define FRONT_TOTAL_TIME_SEC 3.0f // 目标：完成单次最大行程的总时间(秒)
#define FRONT_ACCEL_TIME_SEC 1.0f // 加速/减速缓冲段的时间(秒)

// 🚨 左右非对称力量调参 (解决左侧偏重问题)：
#define FRONT_LEFT_MOVING_MAX_OUT 12000.0f // 左腿(重)：放宽限幅，允许拉到 12000
#define FRONT_RIGHT_MOVING_MAX_OUT 7000.0f // 右腿(轻)：保持原来的健康参数
#define FRONT_LEFT_STOP_MAX_OUT 6000.0f    // 驻车防掉兜底加大
#define FRONT_RIGHT_STOP_MAX_OUT 4000.0f

// ==================== 【后腿（滚珠丝杠+齿轮组） 2.0秒极速参数】 ====================
#define REAR_TOTAL_TIME_SEC 3.0f     // 坚决贯彻 2.0 秒！
#define REAR_ACCEL_TIME_SEC 1.0f     // 丝杠起步快，给 0.4 秒爆发加速
#define REAR_MOVING_MAX_OUT 12000.0f // 突破静摩擦力的狂暴输出！
#define REAR_STOP_MAX_OUT 0.0f       // 驻车力，靠齿轮摩擦力锁死即可

// ==================== 【兑换模式（极慢无级调节）专属参数】 ====================
#define EXCHANGE_TOTAL_TIME_SEC 20.0f // 兑换模式：虚拟总时间 (数字越大，底盘升降越平滑缓慢)
#define EXCHANGE_ACCEL_TIME_SEC 4.0f  // 兑换模式：加速/减速缓冲时间

// ==================== 【硬件基础信息】 ====================
#define CALI_TASK_FREQ 500.0f  // 标定任务运行频率 (Hz) ，在ostask里得知
#define GEAR_RATIO_REAR 1.0f   // 后腿减速比
#define GEAR_RATIO_FRONT 19.0f // 前腿减速比

// ==================== 【标定行为期望 (物理参数)】 ====================
// 🚨 新增：前后腿独立的标定堵转电流阈值 (满载为 16384)
#define FRONT_CALI_STALL_CURRENT 3000.0f // 前腿(齿条)：机械优势小，给大点电流才能判定撞墙
#define REAR_CALI_STALL_CURRENT 5000.0f  // 后腿(丝杠)：推力极其恐怖，阈值给小一点，撞墙瞬间温柔停机防损坏

#define SPEED_RETRACT_REAR_DEG 1200.0f
#define SPEED_EXTEND_REAR_DEG 1200.0f
#define SPEED_RETRACT_FRONT_DEG 108.0f
#define SPEED_EXTEND_FRONT_DEG 108.0f

#define FORCE_ZERO_REAR_DEG 1000.0f
#define FORCE_MAX_REAR_DEG 1500.0f
#define FORCE_ZERO_FRONT_DEG 135.0f
#define FORCE_MAX_FRONT_DEG 600.0f

#define JITTER_TOLERANCE_REAR 15.0f
#define JITTER_TOLERANCE_FRONT 15.0f

#define CALI_TIMEOUT_SEC 40.0f
#define ZERO_CHECK_SEC 1.5f
#define MAX_CHECK_SEC 3.0f
#define MAX_CALI_SAFE_RATIO 0.99f

#define DEG_TO_TICKS(deg, ratio) ((deg) * (ratio) * 8192.0f / 360.0f)
#define CALI_TIMEOUT_TICKS (uint32_t)(CALI_TIMEOUT_SEC * CALI_TASK_FREQ)
#define ZERO_CALI_CHECK_TICKS (uint32_t)(ZERO_CHECK_SEC * CALI_TASK_FREQ)
#define MAX_CALI_CHECK_TICKS (uint32_t)(MAX_CHECK_SEC * CALI_TASK_FREQ)

#define ZERO_CALI_STEP_REAR (DEG_TO_TICKS(SPEED_RETRACT_REAR_DEG, GEAR_RATIO_REAR) / CALI_TASK_FREQ)
#define ZERO_CALI_STEP_FRONT (DEG_TO_TICKS(SPEED_RETRACT_FRONT_DEG, GEAR_RATIO_FRONT) / CALI_TASK_FREQ)
#define MAX_CALI_STEP_REAR (DEG_TO_TICKS(SPEED_EXTEND_REAR_DEG, GEAR_RATIO_REAR) / CALI_TASK_FREQ)
#define MAX_CALI_STEP_FRONT (DEG_TO_TICKS(SPEED_EXTEND_FRONT_DEG, GEAR_RATIO_FRONT) / CALI_TASK_FREQ)

#define ZERO_CALI_SLIP_LIMIT_REAR DEG_TO_TICKS(FORCE_ZERO_REAR_DEG, GEAR_RATIO_REAR)
#define ZERO_CALI_SLIP_LIMIT_FRONT DEG_TO_TICKS(FORCE_ZERO_FRONT_DEG, GEAR_RATIO_FRONT)
#define MAX_CALI_SLIP_LIMIT_REAR DEG_TO_TICKS(FORCE_MAX_REAR_DEG, GEAR_RATIO_REAR)
#define MAX_CALI_SLIP_LIMIT_FRONT DEG_TO_TICKS(FORCE_MAX_FRONT_DEG, GEAR_RATIO_FRONT)

#define ZERO_CALI_STOP_THRES_REAR DEG_TO_TICKS(JITTER_TOLERANCE_REAR *ZERO_CHECK_SEC, GEAR_RATIO_REAR)
#define ZERO_CALI_STOP_THRES_FRONT DEG_TO_TICKS(JITTER_TOLERANCE_FRONT *ZERO_CHECK_SEC, GEAR_RATIO_FRONT)
#define MAX_CALI_STOP_THRES_REAR DEG_TO_TICKS(JITTER_TOLERANCE_REAR *MAX_CHECK_SEC, GEAR_RATIO_REAR)
#define MAX_CALI_STOP_THRES_FRONT DEG_TO_TICKS(JITTER_TOLERANCE_FRONT *MAX_CHECK_SEC, GEAR_RATIO_FRONT)

/* Private variables ---------------------------------------------------------*/
static ChassisInstance *chassis;
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;
static Chassis_Param_s chassis_param;
static float chassis_vx, chassis_vy;
static float vt_lf, vt_rf, vt_lb, vt_rb;
static float lf_radius;
static float rf_radius;
static float lb_radius;
static float rb_radius;
static PIDInstance follow_pid;
static float k0, k1, k2, k3, k4, k5;
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
static void LiftLeg_UpdateSpeed(LiftLeg_t *leg, float total_time, float acc_time, float stroke);
ChassisInstance *ChassisInit(Chassis_Init_Config_s *chassis_init_config);
static void LiftLeg_Init(LiftLeg_t *leg, float *ff_ch, uint8_t use_curve, float total_time, float acc_time,
                         float stroke, float move_out, float stop_out);
static void LiftLeg_SetTarget(LiftLeg_t *leg, float target);
static void LiftLeg_Execute(LiftLeg_t *leg);
void ChassisTask();
void Leg_FSM();

/* Private user code ---------------------------------------------------------*/
ChassisInstance *ChassisInit(Chassis_Init_Config_s *chassis_init_config)
{
    ChassisInstance *chassis_instance = (ChassisInstance *)zmalloc(sizeof(ChassisInstance));

    chassis_param = chassis_init_config->chassis_param;

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

        chassis_instance->front_legs[i].motor->motor_settings.feedforward_flag = SPEED_FEEDFORWARD;
        chassis_instance->front_legs[i].motor->motor_controller.speed_feedforward_ptr = &lift_speed_feedforward[i + 2];

        chassis_instance->rear_legs[i].motor->motor_settings.feedforward_flag = SPEED_FEEDFORWARD;
        chassis_instance->rear_legs[i].motor->motor_controller.speed_feedforward_ptr = &lift_speed_feedforward[i];
    }

    chassis = chassis_instance;
    chassis_ctrl_cmd = &chassis->chassis_ctrl_cmd;
    chassis_ctrl_cmd->backward_lift_in = chassis_param.backward_lift_in;
    chassis_ctrl_cmd->backward_lift_out = chassis_param.backward_lift_out;
    chassis_ctrl_cmd->forward_lift_out = chassis_param.forward_lift_out;
    chassis_ctrl_cmd->forward_lift_in = chassis_param.forward_lift_in;

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
        for (int i = 0; i < 4; i++)
            DJIMotorStop(chassis->wheel_motor[i]);
        for (int i = 0; i < 2; i++)
        {
            DJIMotorEnable(chassis->front_legs[i].motor);
            DJIMotorEnable(chassis->rear_legs[i].motor);
        }
        ChassisCalibrationTask();
        return;
    }

    if (!chassis->cali_state.is_max_calibrated && chassis->cali_state.all_cali_done)
    {
        if (chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_BOTH_EXTEND)
        {
            for (int i = 0; i < 4; i++)
                DJIMotorStop(chassis->wheel_motor[i]);
            for (int i = 0; i < 2; i++)
            {
                DJIMotorEnable(chassis->front_legs[i].motor);
                DJIMotorEnable(chassis->rear_legs[i].motor);
            }
            MaxExtensionCalibrationTask(0);
            return;
        }
        else
        {
            // 用户中途切走了模式（主动取消标定）！
            MaxExtensionCalibrationTask(1); // 传入 1：触发任务内部重置清零！

            // 🚨 修复 2：如果是因为急停触发的打断，绝对不许使能电机！防止 500Hz 疯狂启停！
            if (chassis_ctrl_cmd->chassis_mode != CHASSIS_POWER_OFF)
            {
                // 安全收腿保护：把腿安全地拉回物理零点，防止掉下来
                for (int i = 0; i < 2; i++)
                {
                    DJIMotorEnable(chassis->front_legs[i].motor);
                    DJIMotorEnable(chassis->rear_legs[i].motor);

                    // 🚨 修复 1：把后腿改回 8000！800 是绝对拉不动丝杠的！
                    chassis->front_legs[i].motor->motor_controller.speed_PID.MaxOut = 8000.0f;
                    chassis->rear_legs[i].motor->motor_controller.speed_PID.MaxOut = 6000.0f;

                    // 强制目标归零
                    DJIMotorSetPIDRef(chassis->front_legs[i].motor, chassis->cali_state.init_angle[2 + i]);
                    DJIMotorSetPIDRef(chassis->rear_legs[i].motor, chassis->cali_state.init_angle[i]);
                }
            }
        }
    }

    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_POWER_OFF)
    {
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
        for (int i = 0; i < 4; i++)
            DJIMotorEnable(chassis->wheel_motor[i]);
        for (int i = 0; i < 2; i++)
        {
            DJIMotorEnable(chassis->front_legs[i].motor);
            DJIMotorEnable(chassis->rear_legs[i].motor);
        }
    }

    switch (chassis_ctrl_cmd->chassis_mode)
    {
    case CHASSIS_FOLLOW:
    case CHASSIS_CLIMB_ALL_RETRACT:
    case CHASSIS_CLIMB_FRONT_RETRACT:
        chassis_ctrl_cmd->wz += PIDCalculate(&follow_pid, chassis_ctrl_cmd->offset_angle, 0);
        break;
    default:
        break;
    }

    // 💥 无条件挂载最新腿部大脑
    Leg_FSM();

    static float sin_theta, cos_theta;
    cos_theta = arm_cos_f32(chassis_ctrl_cmd->offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_ctrl_cmd->offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_ctrl_cmd->vx * cos_theta - chassis_ctrl_cmd->vy * sin_theta;
    chassis_vy = chassis_ctrl_cmd->vx * sin_theta + chassis_ctrl_cmd->vy * cos_theta;

    EstimateSpeed();
    MecanumCalculate();
    LimitChassisOutput();
}

/**
 * @brief 梯形曲线更新器
 */
static void Planner_Update(TrapezoidalPlanner_t *planner)
{
    float remain_dist = fabsf(planner->target_pos - planner->current_ref);

    // 🚨 架构级救命补丁：防零除死机！
    // 如果加速度几乎为0（比如被屏蔽的腿，行程为0），说明根本不需要运动。
    // 直接让它乖乖待在原地并退出，绝不能往下做除法！
    if (planner->accel <= 0.0001f)
    {
        planner->current_vel = 0.0f;
        planner->current_ref = planner->target_pos;
        planner->ff_speed = 0.0f;
        planner->is_moving = 0;
        return;
    }

    // 只有安全了，才允许执行除法
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
 * @brief 动态重写腿部的梯形曲线速度与加速度
 */
static void LiftLeg_UpdateSpeed(LiftLeg_t *leg, float total_time, float acc_time, float stroke)
{
    if (leg->use_curve)
    {
        if (acc_time >= total_time / 2.0f)
            acc_time = total_time * 0.3f;

        float v_max = fabsf(stroke) / (total_time - acc_time);
        if (v_max > 42000.0f)
            v_max = 42000.0f; // 硬件安全限幅

        leg->planner.max_vel = v_max / CALI_TASK_FREQ;
        leg->planner.accel = leg->planner.max_vel / (acc_time * CALI_TASK_FREQ);
    }
}

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
        if (acc_time >= total_time / 2.0f)
        {
            acc_time = total_time * 0.3f;
        }
        float v_max = fabsf(stroke) / (total_time - acc_time);
        if (v_max > 42000.0f)
            v_max = 42000.0f;
        leg->planner.max_vel = v_max / CALI_TASK_FREQ;
        leg->planner.accel = leg->planner.max_vel / (acc_time * CALI_TASK_FREQ);
    }
}

static void LiftLeg_SetTarget(LiftLeg_t *leg, float target)
{
    leg->target_pos = target;
    leg->planner.target_pos = target;
}

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

static void MecanumCalculate()
{
    vt_lf = -chassis_vx - chassis_vy - chassis_ctrl_cmd->wz * lf_radius;
    vt_rf = -chassis_vx + chassis_vy - chassis_ctrl_cmd->wz * rf_radius;
    vt_lb = chassis_vx - chassis_vy - chassis_ctrl_cmd->wz * lb_radius;
    vt_rb = chassis_vx + chassis_vy - chassis_ctrl_cmd->wz * rb_radius;
}

static void PowerControl()
{
    float motor_speed_fdb[4];
    for (int i = 0; i < 4; i++)
    {
        motor_speed_fdb[i] = (float)chassis->wheel_motor[i]->measure.speed_aps / 6.f;
    }

    float motor_current_list[4];
    for (int i = 0; i < 4; i++)
    {
        motor_current_list[i] = (float)chassis->wheel_motor[i]->motor_controller.final_output;
    }

    float initial_give_power[4] = {0.0f};
    float initial_total_power = 0.0f;

    for (int i = 0; i < 4; i++)
    {
        initial_give_power[i] =
            k0 + k1 * motor_current_list[i] / (16384.0f / 20.0f) + k2 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
            k3 * motor_current_list[i] / (16384.0f / 20.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
            k4 * motor_current_list[i] / (16384.0f / 20.0f) * motor_current_list[i] / (16384.0f / 20.0f) +
            k5 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f);

        if (initial_give_power[i] > 0)
        {
            initial_total_power += initial_give_power[i];
        }
    }

    if (initial_total_power > (float)chassis_ctrl_cmd->max_power)
    {
        float power_scale = (float)chassis_ctrl_cmd->max_power / initial_total_power;
        float scaled_give_power[4];
        for (int i = 0; i < 4; i++)
        {
            scaled_give_power[i] = initial_give_power[i] * power_scale;
        }

        for (int i = 0; i < 4; i++)
        {
            float a = k4 / (16384.0f / 20.0f) / (16384.0f / 20.0f);
            float b = k1 / (16384.0f / 20.0f) + k3 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) / (16384.0f / 20.0f);
            float c = k2 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
                      k5 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f) -
                      scaled_give_power[i] + k0;
            float discriminant = b * b - 4 * a * c;
            if (discriminant >= 0)
            {
                float sqrt_disc = sqrtf(discriminant);
                float temp1 = (-b + sqrt_disc) / (2 * a);
                float temp2 = (-b - sqrt_disc) / (2 * a);

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
 * @brief 腿部总控状态机 (融合兑换极慢无级调节 与 爬楼梯迅捷切高度)
 */
void Leg_FSM()
{
    static uint8_t is_legs_assembled = 0;
    static uint8_t last_robot_mode = 255;

    // 标定未完全结束前，不接管腿部
    if (!chassis->cali_state.all_cali_done || !chassis->cali_state.is_max_calibrated)
        return;

    float stroke_front_l =
        fabsf(chassis->cali_state.max_angle[2] - chassis->cali_state.init_angle[2]) * MAX_CALI_SAFE_RATIO;
    float stroke_front_r =
        fabsf(chassis->cali_state.max_angle[3] - chassis->cali_state.init_angle[3]) * MAX_CALI_SAFE_RATIO;
    float stroke_rear_l =
        fabsf(chassis->cali_state.max_angle[0] - chassis->cali_state.init_angle[0]) * MAX_CALI_SAFE_RATIO;
    float stroke_rear_r =
        fabsf(chassis->cali_state.max_angle[1] - chassis->cali_state.init_angle[1]) * MAX_CALI_SAFE_RATIO;

    if (!is_legs_assembled)
    {
        LiftLeg_Init(&chassis->front_legs[LEFT], &lift_speed_feedforward[2], 1, FRONT_TOTAL_TIME_SEC,
                     FRONT_ACCEL_TIME_SEC, stroke_front_l, FRONT_LEFT_MOVING_MAX_OUT, FRONT_LEFT_STOP_MAX_OUT);
        LiftLeg_Init(&chassis->front_legs[RIGHT], &lift_speed_feedforward[3], 1, FRONT_TOTAL_TIME_SEC,
                     FRONT_ACCEL_TIME_SEC, stroke_front_r, FRONT_RIGHT_MOVING_MAX_OUT, FRONT_RIGHT_STOP_MAX_OUT);
        LiftLeg_Init(&chassis->rear_legs[LEFT], &lift_speed_feedforward[0], 1, REAR_TOTAL_TIME_SEC, REAR_ACCEL_TIME_SEC,
                     stroke_rear_l, REAR_MOVING_MAX_OUT, REAR_STOP_MAX_OUT);
        LiftLeg_Init(&chassis->rear_legs[RIGHT], &lift_speed_feedforward[1], 1, REAR_TOTAL_TIME_SEC,
                     REAR_ACCEL_TIME_SEC, stroke_rear_r, REAR_MOVING_MAX_OUT, REAR_STOP_MAX_OUT);
        is_legs_assembled = 1;
    }

    // 🌟 核心突破：大模式切换时，动态改变电机的梯形曲线特性
    if (chassis_ctrl_cmd->robot_mode != last_robot_mode)
    {
        if (chassis_ctrl_cmd->robot_mode == 2) // ROBOT_EXCHANGE_MODE (兑换模式)
        {
            // 注入极慢速平滑基因 (使用宏定义，方便调参)
            LiftLeg_UpdateSpeed(&chassis->front_legs[LEFT], EXCHANGE_TOTAL_TIME_SEC, EXCHANGE_ACCEL_TIME_SEC,
                                stroke_front_l);
            LiftLeg_UpdateSpeed(&chassis->front_legs[RIGHT], EXCHANGE_TOTAL_TIME_SEC, EXCHANGE_ACCEL_TIME_SEC,
                                stroke_front_r);
            LiftLeg_UpdateSpeed(&chassis->rear_legs[LEFT], EXCHANGE_TOTAL_TIME_SEC, EXCHANGE_ACCEL_TIME_SEC,
                                stroke_rear_l);
            LiftLeg_UpdateSpeed(&chassis->rear_legs[RIGHT], EXCHANGE_TOTAL_TIME_SEC, EXCHANGE_ACCEL_TIME_SEC,
                                stroke_rear_r);
        }
        else // ROBOT_CLIMB_MODE 或 正常行车
        {
            // 恢复原厂迅捷爆发基因 (直接切画面)
            LiftLeg_UpdateSpeed(&chassis->front_legs[LEFT], FRONT_TOTAL_TIME_SEC, FRONT_ACCEL_TIME_SEC, stroke_front_l);
            LiftLeg_UpdateSpeed(&chassis->front_legs[RIGHT], FRONT_TOTAL_TIME_SEC, FRONT_ACCEL_TIME_SEC,
                                stroke_front_r);
            LiftLeg_UpdateSpeed(&chassis->rear_legs[LEFT], REAR_TOTAL_TIME_SEC, REAR_ACCEL_TIME_SEC, stroke_rear_l);
            LiftLeg_UpdateSpeed(&chassis->rear_legs[RIGHT], REAR_TOTAL_TIME_SEC, REAR_ACCEL_TIME_SEC, stroke_rear_r);
        }
        last_robot_mode = chassis_ctrl_cmd->robot_mode;
    }

    // 🎯 高度指令派发
    if (chassis_ctrl_cmd->robot_mode == 2) // ROBOT_EXCHANGE_MODE
    {
        // 兑换模式：无级调节 (lift_ratio 0~1 映射到 物理行程)
        float ratio = chassis_ctrl_cmd->lift_ratio;
        if (ratio < 0.0f)
            ratio = 0.0f;
        if (ratio > 1.0f)
            ratio = 1.0f; // 限幅保护

        LiftLeg_SetTarget(&chassis->front_legs[LEFT], chassis->cali_state.init_angle[2] + stroke_front_l * ratio);
        LiftLeg_SetTarget(&chassis->front_legs[RIGHT], chassis->cali_state.init_angle[3] + stroke_front_r * ratio);
        LiftLeg_SetTarget(&chassis->rear_legs[LEFT], chassis->cali_state.init_angle[0] + stroke_rear_l * ratio);
        LiftLeg_SetTarget(&chassis->rear_legs[RIGHT], chassis->cali_state.init_angle[1] + stroke_rear_r * ratio);
    }
    else // ROBOT_CLIMB_MODE
    {
        // 爬楼梯模式：遵循原有的离散小状态，迅捷直接切位置
        float front_l_target = chassis->cali_state.init_angle[2] + stroke_front_l;
        float front_r_target = chassis->cali_state.init_angle[3] + stroke_front_r;
        float rear_l_target = chassis->cali_state.init_angle[0] + stroke_rear_l;
        float rear_r_target = chassis->cali_state.init_angle[1] + stroke_rear_r;

        switch (chassis_ctrl_cmd->chassis_mode)
        {
        case CHASSIS_CLIMB_BOTH_EXTEND:
            LiftLeg_SetTarget(&chassis->front_legs[LEFT], front_l_target);
            LiftLeg_SetTarget(&chassis->front_legs[RIGHT], front_r_target);
            LiftLeg_SetTarget(&chassis->rear_legs[LEFT], rear_l_target);
            LiftLeg_SetTarget(&chassis->rear_legs[RIGHT], rear_r_target);
            break;
        case CHASSIS_CLIMB_FRONT_RETRACT:
            LiftLeg_SetTarget(&chassis->front_legs[LEFT], chassis->cali_state.init_angle[2]);
            LiftLeg_SetTarget(&chassis->front_legs[RIGHT], chassis->cali_state.init_angle[3]);
            LiftLeg_SetTarget(&chassis->rear_legs[LEFT], rear_l_target);
            LiftLeg_SetTarget(&chassis->rear_legs[RIGHT], rear_r_target);
            break;
        default: // 行车模式 / 全部收回
            LiftLeg_SetTarget(&chassis->front_legs[LEFT], chassis->cali_state.init_angle[2]);
            LiftLeg_SetTarget(&chassis->front_legs[RIGHT], chassis->cali_state.init_angle[3]);
            LiftLeg_SetTarget(&chassis->rear_legs[LEFT], chassis->cali_state.init_angle[0]);
            LiftLeg_SetTarget(&chassis->rear_legs[RIGHT], chassis->cali_state.init_angle[1]);
            break;
        }
    }
}

/**
 * @brief 阶段一：底盘抬升零点标定任务
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
                    float actual_current = fabsf((float)motor->measure.real_current);

                    // 👇 动态获取前后腿独立的标定堵转阈值
                    float stall_current_thres = (i < 2) ? REAR_CALI_STALL_CURRENT : FRONT_CALI_STALL_CURRENT;

                    if (actual_diff < check_threshold && actual_current > stall_current_thres)
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
 */
static void MaxExtensionCalibrationTask(uint8_t abort_flag)
{
    if (chassis->cali_state.is_max_calibrated)
        return;

    static uint32_t timeout_cnt = 0;
    static float cali_target_angle[4] = {0};
    static float last_check_angle[4] = {0};
    static uint8_t first_run = 1;
    static uint32_t startup_grace_cnt = 0;

    if (abort_flag)
    {
        timeout_cnt = 0;
        first_run = 1;
        startup_grace_cnt = 0;
        for (int i = 0; i < 4; i++)
        {
            chassis->cali_state.max_cali_done[i] = 0;
            max_cali_block_cnt[i] = 0;
        }
        return;
    }

    timeout_cnt++;
    if (timeout_cnt > CALI_TIMEOUT_TICKS)
    {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        for (int i = 0; i < 2; i++)
        {
            DJIMotorSetRef(chassis->rear_legs[i].motor, 0);
            DJIMotorSetRef(chassis->front_legs[i].motor, 0);
        }
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
        }

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
            // 获取当前已经走过的行程
            float current_stroke = fabsf(current_angle - chassis->cali_state.init_angle[i]);

            // 👇 核心绝招：动态推力限幅 (分段给力)
            if (i == 0 || i == 1)
            { // 针对后腿(丝杠)
                if (current_stroke < 2000.0f)
                {
                    // 起步破冰区：给极其狂暴的电流，挣脱静摩擦和原点自锁
                    motor->motor_controller.speed_PID.MaxOut = 15000.0f;
                }
                else
                {
                    // 巡航与摸墙区：破冰后立刻收力，温柔滑行，防止撞坏限位
                    motor->motor_controller.speed_PID.MaxOut = 9500.0f;
                }
            }
            else
            { // 针对前腿(齿条)
                if (current_stroke < 2000.0f)
                {
                    motor->motor_controller.speed_PID.MaxOut = 12000.0f;
                }
                else
                {
                    motor->motor_controller.speed_PID.MaxOut = 7000.0f;
                }
            }

            // 虚拟弹簧：限制目标超前量
            float slip_threshold = (i < 2) ? MAX_CALI_SLIP_LIMIT_REAR : MAX_CALI_SLIP_LIMIT_FRONT;
            if (cali_target_angle[i] > current_angle + slip_threshold)
            {
                cali_target_angle[i] = current_angle + slip_threshold;
            }

            DJIMotorSetPIDRef(motor, cali_target_angle[i]);

            if (startup_grace_cnt >= MAX_CALI_CHECK_TICKS)
            {
                max_cali_block_cnt[i]++;
                if (max_cali_block_cnt[i] > MAX_CALI_CHECK_TICKS)
                {
                    float check_threshold = (i < 2) ? MAX_CALI_STOP_THRES_REAR : MAX_CALI_STOP_THRES_FRONT;
                    float actual_diff = fabsf(current_angle - last_check_angle[i]);
                    float actual_current = fabsf((float)motor->measure.real_current);

                    float stall_current_thres = (i < 2) ? REAR_CALI_STALL_CURRENT : FRONT_CALI_STALL_CURRENT;

                    if (actual_diff < check_threshold && actual_current > stall_current_thres)
                    {
                        uint8_t allow_stop = 1;

                        // 双重保险：在破冰区内，无论怎么堵转都绝对不允许停下！
                        if (i == 0 || i == 1)
                        {
                            if (current_stroke < 10000.0f)
                                allow_stop = 0;
                        }
                        else if (i == 2 || i == 3)
                        {
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
            DJIMotorSetPIDRef(motor, cali_target_angle[i]);
        }
    }

    if (current_all_done)
    {
        float rear_stroke_l = fabsf(chassis->cali_state.max_angle[0] - chassis->cali_state.init_angle[0]);
        float rear_stroke_r = fabsf(chassis->cali_state.max_angle[1] - chassis->cali_state.init_angle[1]);
        float front_stroke_l = fabsf(chassis->cali_state.max_angle[2] - chassis->cali_state.init_angle[2]);
        float front_stroke_r = fabsf(chassis->cali_state.max_angle[3] - chassis->cali_state.init_angle[3]);

        chassis_ctrl_cmd->backward_lift_out = fmaxf(rear_stroke_l, rear_stroke_r);
        chassis_ctrl_cmd->forward_lift_out = fmaxf(front_stroke_l, front_stroke_r);

        chassis->cali_state.is_max_calibrated = 1;
        first_run = 1;
    }
}

/**
 * @brief 预测电机功率并进行限制
 */
static void LimitChassisOutput()
{
    DJIMotorSetPIDRef(chassis->wheel_motor[0], vt_lf);
    DJIMotorSetPIDRef(chassis->wheel_motor[1], vt_rf);
    DJIMotorSetPIDRef(chassis->wheel_motor[2], vt_lb);
    DJIMotorSetPIDRef(chassis->wheel_motor[3], vt_rb);

    if (chassis->cali_state.all_cali_done && chassis->cali_state.is_max_calibrated)
    {
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
 */
static void EstimateSpeed()
{
}