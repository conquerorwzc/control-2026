
#include "chassis.h"

#include "arm_math.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "user_lib.h"
#include "remote_control.h"
/* Private macro -------------------------------------------------------------*/
#define LEFT 0
#define RIGHT 1

// ==================== 【硬件基础信息】 ====================
#define CALI_TASK_FREQ          500.0f  // 标定任务运行频率 (Hz) ，在ostask里得知
#define GEAR_RATIO_REAR         19.0f   // 后腿减速比
#define GEAR_RATIO_FRONT        19.0f   // 前腿减速比

// ==================== 【标定行为期望 (物理参数)】 ====================
// 2.1 速度期望 (输出轴物理角速度：度/秒)
// 后腿: 20mm/s * 90度/mm = 1800 度/秒
#define SPEED_RETRACT_REAR_DEG  3600.0f
#define SPEED_EXTEND_REAR_DEG   3600.0f
// 前腿: 20mm/s * 2.7度/mm = 54 度/秒
#define SPEED_RETRACT_FRONT_DEG 108.0f
#define SPEED_EXTEND_FRONT_DEG  108.0f

// 2.2 力量离合期望 (虚拟弹簧允许拉伸的最大输出轴度数，度数越大爆发的推力越大)
// 后腿 (丝杠): 机械优势极大，允许 2~5mm 误差即可爆发出满载推力
#define FORCE_ZERO_REAR_DEG     1000.0f   // 收缩靠墙: 允许 2mm 误差 (2*90)
#define FORCE_MAX_REAR_DEG      450.0f   // 撑起车身: 允许 5mm 误差 (5*90)

// 前腿 (齿条): 机械优势小，需要允许 5~15mm 误差让 PID 积攒出足够大的电流
#define FORCE_ZERO_FRONT_DEG    135.0f    // 收缩靠墙: 允许 5mm 误差 (5*2.7)
#define FORCE_MAX_FRONT_DEG     40.5f    // 撑起车身: 允许 15mm 误差 (15*2.7)

// 2.3 堵转静止判定期望 (允许的最大抖动速度：度/秒)
#define JITTER_TOLERANCE_REAR   30.0f    // 后腿 1mm 对应的度数
#define JITTER_TOLERANCE_FRONT  2.7f     // 前腿 1mm 对应的度数

// 2.4 判定时间窗口与安全系数
#define CALI_TIMEOUT_SEC        40.0f   // 全局防暴走超时保护时间 (秒)
#define ZERO_CHECK_SEC          2.5f    // 零点堵转持续判定时间 (秒)
#define MAX_CHECK_SEC           1.0f    // 伸展堵转持续判定时间 (秒)
#define MAX_CALI_SAFE_RATIO     0.99f   // 机械限位保留系数

// 公式：度数 转化为 对应电机的 Ticks (1圈=360度=8192*减速比)
#define DEG_TO_TICKS(deg, ratio)      ((deg) * (ratio) * 8192.0f / 360.0f)

// 1. 时间转 Tick 循环次数
#define CALI_TIMEOUT_TICKS            (uint32_t)(CALI_TIMEOUT_SEC * CALI_TASK_FREQ)
#define ZERO_CALI_CHECK_TICKS         (uint32_t)(ZERO_CHECK_SEC * CALI_TASK_FREQ)
#define MAX_CALI_CHECK_TICKS          (uint32_t)(MAX_CHECK_SEC * CALI_TASK_FREQ)

// 2. 斜坡步长推导 (Step = 期望速度转成的Ticks / 频率)
#define ZERO_CALI_STEP_REAR           (DEG_TO_TICKS(SPEED_RETRACT_REAR_DEG, GEAR_RATIO_REAR) / CALI_TASK_FREQ)
#define ZERO_CALI_STEP_FRONT          (DEG_TO_TICKS(SPEED_RETRACT_FRONT_DEG, GEAR_RATIO_FRONT) / CALI_TASK_FREQ)
#define MAX_CALI_STEP_REAR            (DEG_TO_TICKS(SPEED_EXTEND_REAR_DEG, GEAR_RATIO_REAR) / CALI_TASK_FREQ)
#define MAX_CALI_STEP_FRONT           (DEG_TO_TICKS(SPEED_EXTEND_FRONT_DEG, GEAR_RATIO_FRONT) / CALI_TASK_FREQ)
// 3. 滑动离合力矩推导 (转化为 Ticks 误差)
#define ZERO_CALI_SLIP_LIMIT_REAR     DEG_TO_TICKS(FORCE_ZERO_REAR_DEG, GEAR_RATIO_REAR)
#define ZERO_CALI_SLIP_LIMIT_FRONT    DEG_TO_TICKS(FORCE_ZERO_FRONT_DEG, GEAR_RATIO_FRONT)
#define MAX_CALI_SLIP_LIMIT_REAR      DEG_TO_TICKS(FORCE_MAX_REAR_DEG, GEAR_RATIO_REAR)
#define MAX_CALI_SLIP_LIMIT_FRONT     DEG_TO_TICKS(FORCE_MAX_FRONT_DEG, GEAR_RATIO_FRONT)
// 4. 堵转静止容差推导 (允许抖动速度 * 判定时间 = 允许的 Ticks 波动范围)
#define ZERO_CALI_STOP_THRES_REAR     DEG_TO_TICKS(JITTER_TOLERANCE_REAR * ZERO_CHECK_SEC, GEAR_RATIO_REAR)
#define ZERO_CALI_STOP_THRES_FRONT    DEG_TO_TICKS(JITTER_TOLERANCE_FRONT * ZERO_CHECK_SEC, GEAR_RATIO_FRONT)
#define MAX_CALI_STOP_THRES_REAR      DEG_TO_TICKS(JITTER_TOLERANCE_REAR * MAX_CHECK_SEC, GEAR_RATIO_REAR)
#define MAX_CALI_STOP_THRES_FRONT     DEG_TO_TICKS(JITTER_TOLERANCE_FRONT * MAX_CHECK_SEC, GEAR_RATIO_FRONT)



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
static float init_angle[4];
static float target_front_pos[2];
static float target_rear_pos[2];
static uint16_t cali_block_cnt[4] = {0, 0, 0, 0};
static uint8_t  cali_done[4]      = {0, 0, 0, 0};
static uint8_t  all_cali_done     = 0;
static const int16_t cali_current = -1000;
static uint8_t  is_max_calibrated      = 0;             // 是否已经完成最大行程标定
static uint16_t max_cali_block_cnt[4]  = {0, 0, 0, 0};
static uint8_t  max_cali_done[4]       = {0, 0, 0, 0};
static float    max_angle[4]           = {0, 0, 0, 0};  // 记录顶到头的极限角度
static float cali_target_angle[4] = {0};
static float last_check_angle[4]  = {0};
static uint8_t first_run = 1;
static uint32_t startup_grace_cnt = 0;
// 伸出标定电流，方向必须与收腿标定电流相反！
static const int16_t max_cali_current  = 1300;
// 统一比例系数，留出 2% 的安全软限位防撞墙
static const float   extend_safe_ratio = 0.98f;
/* Private function prototypes -----------------------------------------------*/
static void MecanumCalculate();
static void PowerControl();
static void EstimateSpeed();
static void LimitChassisOutput();
static void ChassisCalibrationTask(void);
static void MaxExtensionCalibrationTask(void);
ChassisInstance *ChassisInit(Chassis_Init_Config_s *chassis_init_config);
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
        chassis_instance->lift_forward_motor[i] = DJIMotorInit(&chassis_init_config->lift_forward_motor_config[i]);
        chassis_instance->lift_backward_motor[i] = DJIMotorInit(&chassis_init_config->lift_backward_motor_config[i]);
    }

    chassis = chassis_instance;

    // 初始化cmd
    chassis_ctrl_cmd = &chassis->chassis_ctrl_cmd;
    chassis_ctrl_cmd->backward_lift_in = chassis_param.backward_lift_in;
    chassis_ctrl_cmd->backward_lift_out = chassis_param.backward_lift_out;
    chassis_ctrl_cmd->forward_lift_out = chassis_param.forward_lift_out;
    chassis_ctrl_cmd->forward_lift_in = chassis_param.forward_lift_in;


    while (chassis->lift_backward_motor[1]->measure.real_current == 0)
    {
        osDelay(10);
    }

    chassis_ctrl_cmd->chassis_mode = CHASSIS_CALIBRATING;

    return chassis_instance;
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_CALIBRATING && RemoteControlIsOnline())
    {
        // 标定期间，轮电机断电，抬升电机上电
        for (int i = 0; i < 4; i++) DJIMotorStop(chassis->wheel_motor[i]);
        for (int i = 0; i < 2; i++) {
            DJIMotorEnable(chassis->lift_forward_motor[i]);
            DJIMotorEnable(chassis->lift_backward_motor[i]);
        }

        // 执行非阻塞标定逻辑
        ChassisCalibrationTask();
        return; // 标定期间，直接 return，不执行下方正常的底盘解算和输出
    }


    // 只有在：处于爬楼模式 + 处于全伸出状态 + 最大行程没标定过 + 零点已经标定
    // 这四个条件同时满足时，才允许进入最大伸展标定
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB &&
            chassis_ctrl_cmd->climb_state == CLIMB_STAGE_BOTH_EXTEND &&
            !is_max_calibrated &&
            all_cali_done)
    {
        for (int i = 0; i < 4; i++) DJIMotorStop(chassis->wheel_motor[i]);
        for (int i = 0; i < 2; i++) {
            DJIMotorEnable(chassis->lift_forward_motor[i]);
            DJIMotorEnable(chassis->lift_backward_motor[i]);
        }
        MaxExtensionCalibrationTask();
        return;
    }

    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_POWER_OFF)
    {
        // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        for (int i = 0; i < 4; i++)
            DJIMotorStop(chassis->wheel_motor[i]);
        for (int i = 0; i < 2; i++)
            DJIMotorStop(chassis->lift_forward_motor[i]);
        for (int i = 0; i < 2; i++)
            DJIMotorStop(chassis->lift_backward_motor[i]);
    }
    else
    {
        // 正常工作
        for (int i = 0; i < 4; i++)
            DJIMotorEnable(chassis->wheel_motor[i]);
        for (int i = 0; i < 2; i++)
            DJIMotorEnable(chassis->lift_forward_motor[i]);
        for (int i = 0; i < 2; i++)
            DJIMotorEnable(chassis->lift_backward_motor[i]);
    }
    // 根据控制模式设定旋转速度
    switch (chassis_ctrl_cmd->chassis_mode)
    {
    case CHASSIS_FOLLOW: // 跟随陀螺仪角度作为旋转
        chassis_ctrl_cmd->wz += PIDCalculate(&follow_pid, chassis_ctrl_cmd->offset_angle, 0);
        break;
    case CHASSIS_CLIMB: // 进入底盘抬升爬楼梯状态
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
 *
 */
void Climb_FSM()
{
    const float MAX_SAFE_REAR_EXTEND  = chassis_param.backward_lift_out; // 后腿绝对物理极限
    const float MAX_SAFE_FRONT_EXTEND = chassis_param.forward_lift_out;  // 前腿绝对物理极限

    if (chassis_ctrl_cmd->backward_lift_out > MAX_SAFE_REAR_EXTEND) {
        chassis_ctrl_cmd->backward_lift_out = MAX_SAFE_REAR_EXTEND;
    }
    if (chassis_ctrl_cmd->forward_lift_out > MAX_SAFE_FRONT_EXTEND) {
        chassis_ctrl_cmd->forward_lift_out = MAX_SAFE_FRONT_EXTEND;
    }

    switch (chassis->chassis_ctrl_cmd.climb_state)
    {

    case CLIMB_STAGE_IDLE:
    case CLIMB_STAGE_ALL_RETRACT:
        // 【状态：全收】
        target_rear_pos[LEFT] = init_angle[0] + chassis_ctrl_cmd->backward_lift_in;  // 后：收 左
        target_rear_pos[RIGHT] = init_angle[1] + chassis_ctrl_cmd->backward_lift_in; // 后：收 右
        target_front_pos[LEFT] = init_angle[2] + chassis_ctrl_cmd->forward_lift_in;  // 前：收 左
        target_front_pos[RIGHT] = init_angle[3] + chassis_ctrl_cmd->forward_lift_in; // 前：收 右

        break;

    case CLIMB_STAGE_BOTH_EXTEND:
        // 【状态：全伸】

        // 前导杆伸出
        target_front_pos[LEFT] = init_angle[2] + chassis_ctrl_cmd->forward_lift_out;
        target_front_pos[RIGHT] = init_angle[3] + chassis_ctrl_cmd->forward_lift_out;

        // 后腿伸出
        target_rear_pos[LEFT] = init_angle[0] + chassis_ctrl_cmd->backward_lift_out;
        target_rear_pos[RIGHT] = init_angle[1] + chassis_ctrl_cmd->backward_lift_out;
        break;

    case CLIMB_STAGE_FRONT_RETRACT:
        // 【状态：前收后伸】
        target_rear_pos[LEFT] = init_angle[0] + chassis_ctrl_cmd->backward_lift_out;  // 后：伸 左
        target_rear_pos[RIGHT] = init_angle[1] + chassis_ctrl_cmd->backward_lift_out; // 后：伸 右
        target_front_pos[LEFT] = init_angle[2] + chassis_ctrl_cmd->forward_lift_in;   // 前：收 左
        target_front_pos[RIGHT] = init_angle[3] + chassis_ctrl_cmd->forward_lift_in;  // 前：收 右
        break;
    }
}
/**
 * @brief 阶段一：底盘抬升零点标定任务 (收缩归零)
 * @note  引入滑动离合防震荡与高精度位移检测，各腿独立保存零点
 */
static void ChassisCalibrationTask(void)
{
    if (all_cali_done) return;

    // 1. 全局超时保护
    static uint32_t timeout_cnt = 0;
    timeout_cnt++;
    if (timeout_cnt > CALI_TIMEOUT_TICKS)
    {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        for (int i = 0; i < 2; i++) {
            DJIMotorSetRef(chassis->lift_backward_motor[i], 0);
            DJIMotorSetRef(chassis->lift_forward_motor[i], 0);
        }
        LOGERROR("[Chassis] Calibration TIMEOUT! Partial data saved.");

        // 【超时重置补丁】：允许遥控器再次发起挑战
        timeout_cnt = 0;
        first_run = 1;
        return;
    }



    // 第一次进入时，目标位置对齐物理位置
    if (first_run)
    {
        timeout_cnt = 0;
        startup_grace_cnt = 0;

        cali_target_angle[0] = chassis->lift_backward_motor[0]->measure.total_angle;
        cali_target_angle[1] = chassis->lift_backward_motor[1]->measure.total_angle;
        cali_target_angle[2] = chassis->lift_forward_motor[0]->measure.total_angle;
        cali_target_angle[3] = chassis->lift_forward_motor[1]->measure.total_angle;

        for(int i = 0; i < 4; i++) {
            last_check_angle[i] = cali_target_angle[i];
            cali_block_cnt[i] = 0;
            // 注意：这里不再清空 cali_done！保留之前已经成功的腿的状态！
        }
        first_run = 0;
    }

    // 启动宽限期
    if (startup_grace_cnt < ZERO_CALI_CHECK_TICKS) startup_grace_cnt++;

    for (int i = 0; i < 4; i++)
    {
        DJIMotorInstance *motor = (i < 2) ? chassis->lift_backward_motor[i] : chassis->lift_forward_motor[i - 2];

        // 如果这条腿还没标定完，继续收缩
        if (!cali_done[i])
        {
            // 匀速收缩斜坡
            float cali_step_size = (i < 2) ? ZERO_CALI_STEP_REAR : ZERO_CALI_STEP_FRONT;
            cali_target_angle[i] -= cali_step_size;

            float current_angle = motor->measure.total_angle;

            // 滑动离合
            float slip_threshold = (i < 2) ? ZERO_CALI_SLIP_LIMIT_REAR : ZERO_CALI_SLIP_LIMIT_FRONT;
            if (cali_target_angle[i] < current_angle - slip_threshold) {
                cali_target_angle[i] = current_angle - slip_threshold;
            }

            DJIMotorSetPIDRef(motor, cali_target_angle[i]);

            // 堵转位移结算
            if (startup_grace_cnt >= ZERO_CALI_CHECK_TICKS)
            {
                cali_block_cnt[i]++;
                if (cali_block_cnt[i] > ZERO_CALI_CHECK_TICKS)
                {
                    float check_threshold = (i < 2) ? ZERO_CALI_STOP_THRES_REAR : ZERO_CALI_STOP_THRES_FRONT;
                    if (fabsf(current_angle - last_check_angle[i]) < check_threshold)
                    {
                        // ========================================================
                        // 【核心修改点：只要卡死，立刻将真实坐标保存到 init_angle】
                        cali_done[i] = 1;
                        init_angle[i] = current_angle;
                        // ========================================================
                    }
                    last_check_angle[i] = current_angle;
                    cali_block_cnt[i] = 0;
                }
            }
        }
        else
        {
            // 对于已经提前收到底的腿，用已经保存好的零点死死锁住防掉落
            DJIMotorSetPIDRef(motor, init_angle[i]);
        }
    }

    // 结算逻辑：只有四个都等于 1 时，才算大功告成
    all_cali_done = cali_done[0] && cali_done[1] && cali_done[2] && cali_done[3];
    if (all_cali_done)
    {
        // 计算目标安全收缩位
        target_rear_pos[0]  = init_angle[0] + chassis_ctrl_cmd->backward_lift_in;
        target_rear_pos[1]  = init_angle[1] + chassis_ctrl_cmd->backward_lift_in;
        target_front_pos[0] = init_angle[2] + chassis_ctrl_cmd->forward_lift_in;
        target_front_pos[1] = init_angle[3] + chassis_ctrl_cmd->forward_lift_in;

        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        first_run = 1;
        timeout_cnt = 0;
        LOGINFO("[Chassis] Zero Calibration fully done!");
    }
}

/**
 * @brief 阶段二：最大伸展行程动态标定任务 (顶出)
 * @note  引入带载滑动离合，无视一切震动误差
 */
static void MaxExtensionCalibrationTask(void)
{
    if (is_max_calibrated) return;

    // 1. 全局超时保护
    static uint32_t timeout_cnt = 0;
    timeout_cnt++;
    if (timeout_cnt > CALI_TIMEOUT_TICKS)
    {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        for (int i = 0; i < 2; i++) {
            DJIMotorSetRef(chassis->lift_backward_motor[i], 0);
            DJIMotorSetRef(chassis->lift_forward_motor[i], 0);
        }
        LOGERROR("[Chassis] Max Ext Calibration TIMEOUT!");
        return;
    }

    static float cali_target_angle[4] = {0};
    static float last_check_angle[4]  = {0};
    static uint8_t first_run = 1;

    if (first_run)
    {
        cali_target_angle[0] = chassis->lift_backward_motor[0]->measure.total_angle;
        cali_target_angle[1] = chassis->lift_backward_motor[1]->measure.total_angle;
        cali_target_angle[2] = chassis->lift_forward_motor[0]->measure.total_angle;
        cali_target_angle[3] = chassis->lift_forward_motor[1]->measure.total_angle;
        for(int i=0; i<4; i++) last_check_angle[i] = cali_target_angle[i];
        first_run = 0;
    }

    static uint32_t startup_grace_cnt = 0;
    if (startup_grace_cnt < MAX_CALI_CHECK_TICKS) startup_grace_cnt++;

    uint8_t current_all_done = 1;

    for (int i = 0; i < 4; i++)
    {
        DJIMotorInstance *motor = (i < 2) ? chassis->lift_backward_motor[i] : chassis->lift_forward_motor[i - 2];

        if (!max_cali_done[i])
        {
            current_all_done = 0;

            // 匀速伸出斜坡 (这里是加号)
            float cali_step_size = (i < 2) ? MAX_CALI_STEP_REAR : MAX_CALI_STEP_FRONT;
            cali_target_angle[i] += cali_step_size;

            float current_angle = motor->measure.total_angle;

            // 滑动离合 (防带载软腿/防死点暴走)：如果目标跑得太前，强行拉回
            float slip_threshold = (i < 2) ? MAX_CALI_SLIP_LIMIT_REAR : MAX_CALI_SLIP_LIMIT_FRONT;
            if (cali_target_angle[i] > current_angle + slip_threshold) {
                cali_target_angle[i] = current_angle + slip_threshold;
            }

            DJIMotorSetPIDRef(motor, cali_target_angle[i]);

            // 堵转位移结算
            if (startup_grace_cnt >= MAX_CALI_CHECK_TICKS)
            {
                max_cali_block_cnt[i]++;
                if (max_cali_block_cnt[i] > MAX_CALI_CHECK_TICKS)
                {
                    float check_threshold = (i < 2) ? MAX_CALI_STOP_THRES_REAR : MAX_CALI_STOP_THRES_FRONT;
                    if (fabsf(current_angle - last_check_angle[i]) < check_threshold)
                    {
                        max_cali_done[i] = 1;
                        max_angle[i] = current_angle;
                    }
                    last_check_angle[i] = current_angle;
                    max_cali_block_cnt[i] = 0;
                }
            }
        }
        else
        {
            // 防软腿脱力保护：已经伸满的腿用 PID 死死锁定最大角度撑住车身！
            DJIMotorSetPIDRef(motor, max_angle[i]);
        }
    }

    // 结算逻辑与安全配置
    if (current_all_done)
    {
        float rear_stroke_l  = fabsf(max_angle[0] - init_angle[0]);
        float rear_stroke_r  = fabsf(max_angle[1] - init_angle[1]);
        float front_stroke_l = fabsf(max_angle[2] - init_angle[2]);
        float front_stroke_r = fabsf(max_angle[3] - init_angle[3]);

        // 木桶效应保护，取最小值
        float rear_min_stroke  = fminf(rear_stroke_l, rear_stroke_r);
        float front_min_stroke = fminf(front_stroke_l, front_stroke_r);

        float rear_sign  = (max_angle[0] > init_angle[0]) ? 1.0f : -1.0f;
        float front_sign = (max_angle[2] > init_angle[2]) ? 1.0f : -1.0f;

        // 应用极限安全系数写入配置
        chassis_ctrl_cmd->backward_lift_out = rear_sign * rear_min_stroke * MAX_CALI_SAFE_RATIO;
        chassis_ctrl_cmd->forward_lift_out  = front_sign * front_min_stroke * MAX_CALI_SAFE_RATIO;

        is_max_calibrated = 1;
        first_run = 1;
        LOGINFO("[Chassis] Max Ext Calibration done! Max parameters saved.");
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

    DJIMotorSetPIDRef(chassis->lift_forward_motor[LEFT], target_front_pos[LEFT]);
    DJIMotorSetPIDRef(chassis->lift_forward_motor[RIGHT], target_front_pos[RIGHT]);
    DJIMotorSetPIDRef(chassis->lift_backward_motor[LEFT], target_rear_pos[LEFT]);
    DJIMotorSetPIDRef(chassis->lift_backward_motor[RIGHT], target_rear_pos[RIGHT]);
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
