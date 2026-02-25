
#include "chassis.h"

#include "arm_math.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "user_lib.h"
#include "remote_control.h"
/* Private macro -------------------------------------------------------------*/
#define LEFT 0
#define RIGHT 1
//@todo:数据待测
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


    while (chassis->lift_backward_motor[0]->measure.real_current == 0)
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
    //如果未标定全伸展尺寸，则进入全伸展的标定
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB &&
        chassis_ctrl_cmd->climb_state == CLIMB_STAGE_BOTH_EXTEND &&
        !is_max_calibrated)
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
 * @brief 非阻塞式的底盘抬升零点标定任务 (引入滑动离合防震荡版)
 */
static void ChassisCalibrationTask(void)
{
    if (all_cali_done) return;

    // 超时保护：拉长到 25 秒 (12500次循环)，绝对够收到底了
    static uint16_t timeout_cnt = 0;
    timeout_cnt++;
    if (timeout_cnt > 12500)
    {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        for (int i = 0; i < 2; i++) {
            DJIMotorSetRef(chassis->lift_backward_motor[i], 0);
            DJIMotorSetRef(chassis->lift_forward_motor[i], 0);
        }
        LOGERROR("[Chassis] Calibration TIMEOUT! Power off for safety!");
        return;
    }

    static float cali_target_angle[4] = {0};
    static float last_check_angle[4]  = {0};
    static float cali_zero_angle[4]   = {0};
    static uint8_t first_run = 1;

    if (first_run)
    {
        cali_target_angle[0] = chassis->lift_backward_motor[0]->measure.total_angle;
        cali_target_angle[1] = chassis->lift_backward_motor[1]->measure.total_angle;
        cali_target_angle[2] = chassis->lift_forward_motor[0]->measure.total_angle;
        cali_target_angle[3] = chassis->lift_forward_motor[1]->measure.total_angle;

        for(int i = 0; i < 4; i++) {
            last_check_angle[i] = cali_target_angle[i];
        }
        first_run = 0;
    }

    static uint16_t startup_grace_cnt = 0;
    if (startup_grace_cnt < 500) startup_grace_cnt++;

    for (int i = 0; i < 4; i++)
    {
        DJIMotorInstance *motor = (i < 2) ? chassis->lift_backward_motor[i] : chassis->lift_forward_motor[i - 2];

        if (!cali_done[i])
        {
            float cali_step_size = (i < 2) ? 200.0f : 10.0f;
            cali_target_angle[i] -= cali_step_size; // 向内收缩

            float current_angle = motor->measure.total_angle;

            // =========================================================
            // 【核心修复：滑动离合（Slip Clutch）防震荡暴走】
            // 如果撞到了死点，实际位置不再减小，绝不让目标位置无底线地跑飞！
            // 强行把两者的最大误差限制在 20000 以内。
            // 这样 PID 会维持一个恒定、安全的拉力，绝不会引发高频震荡导致误判！
            if (cali_target_angle[i] < current_angle - 20000.0f) {
                cali_target_angle[i] = current_angle - 20000.0f;
            }
            // =========================================================

            DJIMotorSetPIDRef(motor, cali_target_angle[i]);

            if (startup_grace_cnt >= 500)
            {
                cali_block_cnt[i]++;

                // 每隔 1.0 秒 结算一次位移
                if (cali_block_cnt[i] > 500)
                {
                    // 既然解决了震荡，这里的位移容差也可以适当放宽，保证稳定触发
                    float check_threshold = (i < 2) ? 15000.0f : 3000.0f;

                    if (fabsf(current_angle - last_check_angle[i]) < check_threshold)
                    {
                        cali_done[i] = 1;
                        cali_zero_angle[i] = current_angle;
                    }

                    last_check_angle[i] = current_angle;
                    cali_block_cnt[i] = 0;
                }
            }
        }
        else
        {
            DJIMotorSetPIDRef(motor, cali_zero_angle[i]);
        }
    }

    all_cali_done = cali_done[0] && cali_done[1] && cali_done[2] && cali_done[3];

    if (all_cali_done)
    {
        init_angle[0] = cali_zero_angle[0];
        init_angle[1] = cali_zero_angle[1];
        init_angle[2] = cali_zero_angle[2];
        init_angle[3] = cali_zero_angle[3];

        target_rear_pos[0]  = init_angle[0] + chassis_ctrl_cmd->backward_lift_in;
        target_rear_pos[1]  = init_angle[1] + chassis_ctrl_cmd->backward_lift_in;
        target_front_pos[0] = init_angle[2] + chassis_ctrl_cmd->forward_lift_in;
        target_front_pos[1] = init_angle[3] + chassis_ctrl_cmd->forward_lift_in;

        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        first_run = 1;
        LOGINFO("[Chassis] Calibration done! Zero points recorded.");
    }
}
/**
 * @brief 最大伸展行程动态标定任务 (位置环斜坡控制 - 经典纯软件堵转检测版)
 */
static void MaxExtensionCalibrationTask(void)
{
    if (is_max_calibrated) return;

    // 超时保护 (给足 25 秒的极限宽裕时间，12500 * 2ms = 25s)
    static uint16_t timeout_cnt = 0;
    timeout_cnt++;
    if (timeout_cnt > 12500)
    {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        for (int i = 0; i < 2; i++) {
            DJIMotorSetRef(chassis->lift_backward_motor[i], 0);
            DJIMotorSetRef(chassis->lift_forward_motor[i], 0);
        }
        return;
    }

    static float cali_target_angle[4] = {0};
    static uint8_t first_run = 1;

    // 第一次进入时，让目标位置对齐当前物理位置，防止起步抽搐
    if (first_run)
    {
        cali_target_angle[0] = chassis->lift_backward_motor[0]->measure.total_angle;
        cali_target_angle[1] = chassis->lift_backward_motor[1]->measure.total_angle;
        cali_target_angle[2] = chassis->lift_forward_motor[0]->measure.total_angle;
        cali_target_angle[3] = chassis->lift_forward_motor[1]->measure.total_angle;
        first_run = 0;
    }

    uint8_t current_all_done = 1;

    for (int i = 0; i < 4; i++)
    {
        DJIMotorInstance *motor = (i < 2) ? chassis->lift_backward_motor[i] : chassis->lift_forward_motor[i - 2];

        if (!max_cali_done[i])
        {
            current_all_done = 0;

            // ================== 参数分离与斜坡配置 ==================
            float cali_step_size;
            float cali_err_threshold;

            if (i < 2) {
                // 【后腿】行程大，允许的带载滞后误差放宽到 50000
                cali_step_size = 300.0f;
                cali_err_threshold = 50000.0f;
            } else {
                // 【前腿】行程小，滞后误差放到 5000
                cali_step_size = 15.0f;
                cali_err_threshold = 5000.0f;
            }
            // ========================================================

            // 1. 目标角度匀速斜坡增加
            cali_target_angle[i] += cali_step_size;

            // 下发位置环指令
            DJIMotorSetPIDRef(motor, cali_target_angle[i]);

            // 2. 获取反馈与误差
            float current_angle = motor->measure.total_angle;
            float current_speed = motor->measure.speed_aps;
            float pos_error = fabsf(cali_target_angle[i] - current_angle);

            // 3. 软件堵转判定核心逻辑
            // 条件 A: 误差拉得足够大 (说明 PID 真拉不动了，排除了起步阻力)
            // 条件 B: 速度小于 60 (放宽对震动的容忍，无视电机受阻时的轻微嗡嗡声)
            if (pos_error > cali_err_threshold && fabsf(current_speed) < 60.0f)
            {
                max_cali_block_cnt[i]++;
                // 必须持续堵转 0.6 秒 (300次)！防止半路的暂时卡涩被当成到底
                if (max_cali_block_cnt[i] > 300)
                {
                    max_cali_done[i] = 1;
                    max_angle[i] = current_angle; // 记录极限物理死点

                    // 【防倒塌机制】标定完成后，立刻把 PID 目标锁死在物理死点，强力支撑车体！
                    DJIMotorSetPIDRef(motor, max_angle[i]);
                }
            }
            else
            {
                // 只要速度恢复，或者误差还没达到阈值(说明PID还在积攒力量推)，就清零计数
                max_cali_block_cnt[i] = 0;
            }
        }
        else
        {
            // 对于已经提前伸满的腿，每一帧都要持续下发 PID 锁死指令，撑住车体！
            DJIMotorSetPIDRef(motor, max_angle[i]);
        }
    }

    // 后续结算逻辑
    if (current_all_done)
    {
        float rear_stroke_l  = fabsf(max_angle[0] - init_angle[0]);
        float rear_stroke_r  = fabsf(max_angle[1] - init_angle[1]);
        float front_stroke_l = fabsf(max_angle[2] - init_angle[2]);
        float front_stroke_r = fabsf(max_angle[3] - init_angle[3]);

        // 木桶效应保护机械干涉
        float rear_min_stroke  = fminf(rear_stroke_l, rear_stroke_r);
        float front_min_stroke = fminf(front_stroke_l, front_stroke_r);

        float rear_sign  = (max_angle[0] > init_angle[0]) ? 1.0f : -1.0f;
        float front_sign = (max_angle[2] > init_angle[2]) ? 1.0f : -1.0f;

        // 保留 99% 的绝对安全限位
        chassis_ctrl_cmd->backward_lift_out = rear_sign * rear_min_stroke * 0.99f;
        chassis_ctrl_cmd->forward_lift_out  = front_sign * front_min_stroke * 0.99f;

        is_max_calibrated = 1;
        first_run = 1; // 标定彻底结束，重置首圈标志
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
