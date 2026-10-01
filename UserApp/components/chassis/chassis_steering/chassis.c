//
// 由 yang6 创建于 2025/11/17.
//
/**
 * @file    chassis.c
 * @author  YZC
 * @author  注释与修改：SRM-Control 2026
 * @date    2026/1/1
 * @copyright Copyright (c) SHU SRM 2026 all rights reserved
 * @brief   舵轮底盘模块
 */
#include "chassis.h"

#include <math.h>

#include "arm_math.h"
#include "referee.h"
#include "user_lib.h"

#define STEERING_CMD_DEADBAND 0.5f
#define STEERING_FLIP_ENTER_ANGLE 100.0f
#define STEERING_FLIP_EXIT_ANGLE 80.0f
#define STEERING_NO_ATTENUATION_ANGLE 8.0f
#define FOLLOW_START_DEADBAND 5.0f
#define FOLLOW_STOP_DEADBAND 2.0f
#define WHEEL_CURRENT_OUT_PER_AMP (16384.0f / 20.0f)
#define RUDDER_CURRENT_OUT_PER_AMP (16384.0f / 3.0f)
#define MOTOR_OUTPUT_LIMIT 16000.0f
#define SUPERCAP_SAFE_POWER 35U
#define SUPERCAP_ACTIVE_POWER 150U
#define SUPERCAP_FULL_VOLTAGE 18.0f
#define SUPERCAP_CHARGE_VOLTAGE 14.0f
#define SUPERCAP_FORCE_CHARGE_VOLTAGE 13.5f
#define SUPERCAP_SAFETY_VOLTAGE 13.0f

/**
 * @brief 既无超级电容、又读不到裁判系统功率时的兜底功率上限。
 *
 * 台架调试(无超电、无裁判系统)会走到这个值。取 80W 与各 robot 层手写的初始
 * max_power 保持一致；若为 0，功率环会把四轮电流直接削到 0，表现为"轮子不动"。
 */
#define CHASSIS_DEFAULT_NO_REFEREE_POWER 80U

/**
 * @brief 单个舵轮模块的目标地面速度向量。
 *
 * x/y 为底盘坐标系下的分量，后续会转换成舵角和轮速。
 */
typedef struct
{
    float x;
    float y;
} WheelVector_s;

/* ==========================================================================================
 *  坐标系与轮组位置约定 —— 这两件事是本文件所有公式的唯一依据
 *
 *  【底盘坐标系】(robot 层与解算共用, 与 offset_angle 无关)
 *      +x = 向右平移          +y = 前进(车头方向)          +wz = 顺时针旋转(俯视)
 *
 *  注: 这不是数学上常见的右手系(+x 前, +y 左, +z 上, 逆时针为正)。本车按队伍习惯
 *      固定为"+y 前 / +x 右 / 顺时针为正", 因此下面所有公式里的 wz 项符号都按此约定书写。
 *
 *  【轮组位置】kSteerPos[i] = 该轮中心相对底盘中心的单位位置矢量 (正 / 负 = ±半轴距或半轮距)
 *
 *      LF = (-a, +b) 左前      RF = (+a, +b) 右前
 *      LB = (-a, -b) 左后      RB = (+a, -b) 右后
 *
 *      (a = track_width/2, b = wheel_base/2, 单位归一化为 1)
 *
 *  【为什么这些正负号不能随便写】
 *      刚体绕中心旋转时, 各轮速度【减去整车平移分量之后】必须落在切向上, 即与位置矢量
 *      垂直。写成不变量就是:
 *
 *          (1) 自转分量 v_i - v̄  必须与 (r_iy, -r_ix) 同向   (v̄ = 四轮速度平均)
 *          (2) 平移分量必须与各轮位置无关(四轮同向)
 *
 *      (r_y, -r_x) 就是 r 顺时针转 90°, 也就是"+wz = 顺时针"时该轮应有的切向。
 *      只要有一个轮子写错, 它就会带上径向分量 —— 这种错误在【纯平移(wz=0)时完全看不出来】,
 *      只有自转才暴露, 表现为某个轮子朝完全错误的方向指。
 *
 *      @attention 这两条不变量已写成脚本断言(见 tests/chassis_steering_kinematics.c,
 *                 跑法见 demo_steering/README.md), 改动下面的符号后必须重新跑一遍
 *                 —— 手算这一点非常容易翻车。
 *
 *  【上板实测的两个开关】若整车方向不对, 先分清是哪一类, 不要乱改公式:
 *      1. 自转方向整体反了(三个平移方向都对) -> 把 kSteerCwSign 取 -1;
 *      2. 自转对了但车"横着走" -> 是舵机零偏标定朝向不对(见 chassis.h)。
 * ========================================================================================== */

/* +wz = 顺时针 时旋转分量取 +1; 若实测自转方向相反, 只改这一个常量 */
#define kSteerCwSign (+1.0f)

/*
 * 舵机编码器计数方向: 1 = 逆时针旋转时 ecd 变大; 0 = 顺时针旋转时 ecd 变大。
 *
 * 【为什么需要它】解算约定 st 随【顺时针】增大, 而编码器 ecd 只提供绝对值,
 * 它究竟随哪个方向变大是硬件/装配决定的。FK 要把编码器角换算成解算角, 就必须知道
 * 这个方向。
 *
 * 【怎么一次测准】用 ChassisGetDebugState() 的 ecd_deg 字段(原始编码器角):
 *     1. 舵机【不使能】(把某个 rudder_motor_offset 留 0 即可), 避免它自己转;
 *     2. 从上往下看(俯视), 用手把某个舵轮【逆时针】慢慢转过一个明显角度;
 *     3. 看 ecd_deg 的变化方向:
 *          ecd_deg 变大 -> 本常量置 1
 *          ecd_deg 变小 -> 本常量置 0
 *
 * 【配错的后果】只影响 FK_Vx / FK_Wz 这两个【调试观测量】(会左右镜像、Wz 反号),
 * 不影响 st 的解算, 也不影响车怎么走 —— 因为 st 是逆解算直接给的, 与编码器计数
 * 方向无关。所以不必为它反复试车。
 *
 * @attention 真正决定"车走不走对"的是舵机反馈的符号, 即 RUDDER_MOTOR_CONFIG 里的
 *            feedback_reverse_flag: 角度环对 total_angle 闭环, 必须让它随
 *            【顺时针】增大。这两处描述的是同一个硬件方向, 判定标准都是
 *            "ece 随顺时针增大吗":
 *                ecd 随顺时针增大(本常量 = 0) -> total_angle 也随顺时针增大 -> feedback_reverse_flag = NORMAL
 *                ecd 随逆时针增大(本常量 = 1) -> 必须取负才能随顺时针增大  -> feedback_reverse_flag = REVERSE
 *            @note 二者取值必须一致, 不要一个 REVERSE 一个 NORMAL。
 */
#define kRudderEncCcwPositive 0

static const WheelVector_s kSteerPos[WHEEL_COUNT] = {
    [LF] = {.x = -1.0f, .y = +1.0f}, // 左前
    [LB] = {.x = -1.0f, .y = -1.0f}, // 左后
    [RB] = {.x = +1.0f, .y = -1.0f}, // 右后
    [RF] = {.x = +1.0f, .y = +1.0f}, // 右前
};

/*
 * 第 i 轮的自转分量: wz·(r_y, -r_x)
 *
 * 推导: 轮心速度要落在【切向】上, 即位置矢量 r 顺时针转 90° 的方向:
 *       (r_x, r_y) 顺时针转 90° -> (r_y, -r_x)
 *       故 v_rot = wz · (r_y, -r_x)  —— 注意【不是】(-r_y, r_x), 两者正好反向。
 * 用 kSteerPos 展开即为:
 *       LF(-1,+1) -> (+wz, -wz)      RF(+1,+1) -> (+wz, -wz)
 *       LB(-1,-1) -> (-wz, -wz)      RB(+1,-1) -> (-wz, +wz)
 */
#define kSteerRotX(i) (+kSteerCwSign * chassis_ctrl_cmd->wz * kSteerPos[i].y)
#define kSteerRotY(i) (-kSteerCwSign * chassis_ctrl_cmd->wz * kSteerPos[i].x)

/*
 * 第 i 轮的 (v_i × r_i) 的 z 分量, 用于正运动学最小二乘反解 wz。
 *
 * 推导: 设 v_i = u + wz·(r_iy, -r_ix) (u 为整车平移), 则
 *       (v_i × r_i)_z = u_x·r_iy - u_y·r_ix + wz·(r_iy² + r_ix²)
 *   对四个轮子求和时, 因位置关于中心对称(Σr_i = 0), u 的两项互相抵消, 于是
 *       Σ(v_i × r_i)_z = wz·Σ|r_i|²,  Σ|r_i|² = 4·(√2)² = 8
 *   =>  wz = Σ(v_i × r_i)_z / 8   —— 与下面 FK 里的 /8.0f 对应, 无需额外负号。
 */
#define SteerCrossZ(i) (omega_x[i] * kSteerPos[i].y - omega_y[i] * kSteerPos[i].x)

static referee_info_t *referee_data;
static ChassisInstance *chassis;
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;
static Power_Param_3508_s wheel_power_param;
static PIDInstance follow_pid;

static float chassis_vx;
static float chassis_vy;
static float wheel_speed_ref[WHEEL_COUNT] = {0.0f};
static float rudder_angle_ref[WHEEL_COUNT] = {0.0f};
static int8_t wheel_direction[WHEEL_COUNT] = {1, 1, 1, 1};
static float omega_x[WHEEL_COUNT] = {0.0f};
static float omega_y[WHEEL_COUNT] = {0.0f};
static uint8_t follow_enabled;

/* 当前生效的底盘功率上限，由 UpdateSuperCapMode() 维护，对外经 ChassisGetPowerLimit() 读取。 */
 uint16_t chassis_power_limit = CHASSIS_DEFAULT_NO_REFEREE_POWER;

/* robot 层设定的功率上限，仅在"没有裁判系统"时作为兜底，见 ChassisSetPowerLimit()。 */
static uint16_t chassis_set_power_limit = CHASSIS_DEFAULT_NO_REFEREE_POWER;

/* 舵角参考值是否已经用实测角播种过，见 SeedRudderReference()。 */
static uint8_t rudder_ref_seeded;

/* 调试/观测变量保持为全局，方便在 watch 窗口里查看。 */
float FK_Vx;
float FK_Vy;
float FK_Wz;
float initial_rudder_total_power = 0.0f;
float initial_rudder_give_power[WHEEL_COUNT] = {0.0f};

/* 小型数学 helper 用来提高功率模型公式的可读性。 */
static float Square(float value)
{
    return value * value;
}

/**
 * @brief 将角度归一化到 [-180, 180]，单位：度。
 */
static float NormalizeAngle180(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    else if (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief 将角度归一化到 [0, 360)，单位：度。
 */
static float NormalizeAngle360(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle < 0.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief 带下限保护地扣除功率降额，避免无符号数下溢。
 */
static uint16_t PowerLimitMinus(uint16_t limit, uint16_t margin)
{
    return limit > margin ? (uint16_t)(limit - margin) : 0U;
}

/**
 * @brief 将裁判系统功率上限裁剪到超电协议字段可表示范围。
 */
static int16_t ClampPowerForSuperCap(uint16_t power_limit)
{
    return (int16_t)(power_limit > UINT8_MAX ? UINT8_MAX : power_limit);
}

/**
 * @brief 裁判系统上报的底盘功率上限; 读到 0 视为"没有可用的功率预算"。
 *
 * 返回 0 表示本周期没有可用预算, 由调用方决定退回哪个兜底值：
 *   - 配置了超电：0 会让状态机维持在 SAFETY_MODE，属于保守的安全行为；
 *   - 没有超电：调用方退回 chassis_set_power_limit，否则功率环会把四轮电流
 *     削到 0，表现为"轮子完全不动"。
 *
 * @note 这里【不区分】"裁判系统未接入"和"裁判系统接入但上报 0", 两者都按
 *       "没有预算"处理。这样组件不需要向 referee 模块索取额外接口(保持模块层
 *       不被改动)。代价是裁判系统真的上报 0 时会退回本车设定值, 但那属于
 *       裁判系统异常/未就绪的情形, 退回设定值是合理且安全的行为。
 */
static uint16_t RefereePowerLimit(void)
{
    return referee_data->GameRobotState.chassis_power_limit;
}

/* DJI 电流指令值 -> 功率拟合模型使用的物理电流估计值。 */
static float CurrentOutputTo3508Amp(float current_output)
{
    return current_output / WHEEL_CURRENT_OUT_PER_AMP;
}

/* DJI 电流指令值 -> GM6020 功率预算使用的电流估计值。 */
static float CurrentOutputTo6020Amp(float current_output)
{
    return current_output / RUDDER_CURRENT_OUT_PER_AMP;
}

/* 电机速度反馈单位是度/秒，功率模型使用弧度/秒。 */
static float WheelSpeedToRadPerSec(float speed_aps)
{
    return speed_aps * DEGREE_2_RAD;
}

/**
 * @brief 将目标舵角转换为最近的等效控制指令。
 *
 * 如果直接打舵需要转过 90 度以上，则反向驱动轮电机，并给舵电机
 * 下发等效的反向舵角，让模块始终走短弧。
 */
static float AngleToOptimalAngle(float target_angle, float current_angle, int8_t *direction)
{
    float diff = NormalizeAngle180(target_angle - current_angle);

    if (*direction > 0)
    {
        if (diff > STEERING_FLIP_ENTER_ANGLE)
        {
            *direction = -1;
            return current_angle + diff - 180.0f;
        }
        if (diff < -STEERING_FLIP_ENTER_ANGLE)
        {
            *direction = -1;
            return current_angle + diff + 180.0f;
        }

        return current_angle + diff;
    }

    // 已经反向驱动时，必须等直接角度误差回到较小范围再恢复正向。
    if (fabsf(diff) < STEERING_FLIP_EXIT_ANGLE)
    {
        *direction = 1;
        return current_angle + diff;
    }

    if (diff >= 0.0f)
    {
        return current_angle + diff - 180.0f;
    }

    return current_angle + diff + 180.0f;
}

/**
 * @brief 判断舵电机是否可以安全使能。
 *
 * GM6020 一使能就会朝"零偏对应的绝对角度"闭环。若某个零偏仍是 0，该目标角度
 * 没有意义，舵轮可能顶到机械限位持续堵转，因此零偏未标定时拒绝使能舵电机。
 */
static uint8_t RudderCalibrationValid(void)
{
    if (chassis == NULL)
    {
        return 0U;
    }

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        if (chassis->rudder_offset[i] == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

/**
 * @brief 使能或停止四个舵电机。
 *
 * @note 零偏未标定时只让舵电机保持停机，轮电机不受影响 —— 台架单轮组调试
 *       仍然可以正常验证轮电机的收发与转动。
 */
static void EnableRudderMotors(uint8_t enable)
{
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        if (enable)
        {
            DJIMotorEnable(chassis->rudder_motor[i]);
        }
        else
        {
            DJIMotorStop(chassis->rudder_motor[i]);
        }
    }
}

/**
 * @brief 以底盘整体为单位使能或停止所有轮电机和舵电机。
 */
static void EnableChassisMotors(uint8_t enable)
{
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        if (enable)
        {
            DJIMotorEnable(chassis->wheel_motor[i]);
        }
        else
        {
            DJIMotorStop(chassis->wheel_motor[i]);
        }
    }

    // 舵电机额外受"零偏是否标定"约束，避免未标定的 GM6020 上电即堵转。
    EnableRudderMotors(enable && RudderCalibrationValid());
}

/**
 * @brief 将轮电机强制配置为速度环，同时保留 PID/CAN 配置。
 */
static void ConfigureWheelMotor(Motor_Init_Config_s *config)
{
    config->controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
    config->controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
    config->controller_setting_init_config.outer_loop_type = SPEED_LOOP;
    config->controller_setting_init_config.close_loop_type = SPEED_LOOP;
}

/**
 * @brief 将舵电机强制配置为角度-速度串级环，同时保留 PID/CAN 配置。
 */
static void ConfigureRudderMotor(Motor_Init_Config_s *config)
{
    config->controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
    config->controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
    config->controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    config->controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;
}

/**
 * @brief 根据舵角和轮速估计底盘速度(正运动学, 仅用于观测)。
 *
 * 与 SteeringCalculate() 共用同一套约定, 两者必须成对成立:
 *   把逆解算算出的 (st_i, |v_i|) 回代本函数, 应当还原出同一个 (vx, vy, wz)。
 *
 * @note 舵角只用单圈编码器 + 零偏得出, 所以物理上只能分辨到 180°:
 *       舵机朝"机械正前方"和"机械正后方"读到的是同一个角度。因此本函数里
 *       "角度 0° = 指向 +y(前)" 是唯一能保证符号正确的假设 —— 这也是为什么
 *       标定零偏时必须把舵轮摆到【机械正前方】。
 *
 * @note 【编码器方向】由 kRudderEncCcwPositive 单独表达, 见该常量上方的说明。
 *       它【只影响本函数】(FK_Vx 的左右、FK_Wz 的正负), 不影响 st 的解算 ——
 *       st 是逆解算直接给出的, 与编码器怎么计数无关。所以这个符号配错只会让调试
 *       打印难读, 不会让车走错。
 *
 *       注意本函数读的是 angle_single_round, 它 = ecd·系数, 【不受
 *       feedback_reverse_flag 影响】(那个 flag 只改 speed_aps / real_current /
 *       total_angle, 见 dji_motor.c 的 DecodeDJIMotor)。而角度环闭环用的是
 *       total_angle(见 DJIMotorControl), 那个是被取过负的。两个量基准不同,
 *       别互相套用。
 *
 * @note FK_Wz 与 wz 同号: 正 = 顺时针(俯视)。
 */
static void ForwardKinematicCal(void)
{
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        // 该轮速度矢量相对 +y(正前方) 的角度: 指向 +x(右) 为 +90°, 顺时针为正。
        // 编码器角 -> 解算角 的换算只差一个方向系数。
        float rudder_angle = chassis->rudder_motor[i]->measure.angle_single_round -
                             (float)chassis->rudder_offset[i] * ECD_ANGLE_COEF_DJI;
        rudder_angle = kRudderEncCcwPositive ? -NormalizeAngle180(rudder_angle) * DEGREE_2_RAD
                                             : NormalizeAngle180(rudder_angle) * DEGREE_2_RAD;

        // st = atan2(vx, vy) 的逆运算 => vx = |v|·sin(st), vy = |v|·cos(st)
        omega_x[i] = chassis->wheel_motor[i]->measure.speed_aps * arm_sin_f32(rudder_angle);
        omega_y[i] = chassis->wheel_motor[i]->measure.speed_aps * arm_cos_f32(rudder_angle);
    }

    FK_Vx = (omega_x[LF] + omega_x[RF] + omega_x[LB] + omega_x[RB]) / 4.0f;
    FK_Vy = (omega_y[LF] + omega_y[RF] + omega_y[LB] + omega_y[RB]) / 4.0f;

    // 最小二乘反解: ω = Σ(v_i × r_i)_z / Σ|r_i|² 。
    // kSteerPos 是归一化的单位位置(模长 √2), 故 Σ|r_i|² = 4·(√2)² = 8。
    FK_Wz = (SteerCrossZ(LF) + SteerCrossZ(LB) + SteerCrossZ(RB) + SteerCrossZ(RF)) / 8.0f;
}

/**
 * @brief 当任意轮速超过上限时，按比例整体缩放四个轮速参考值。
 */
static void LimitWheelSpeed(void)
{
    float max_speed = 0.0f;

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        float speed_abs = fabsf(wheel_speed_ref[i]);
        if (max_speed < speed_abs)
        {
            max_speed = speed_abs;
        }
    }

    if (max_speed > MAX_WHEEL_SPEED)
    {
        float scale = MAX_WHEEL_SPEED / max_speed;
        for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
        {
            wheel_speed_ref[i] *= scale;
        }
    }
}

/**
 * @brief 小陀螺模式下, 按自转已经占用的轮速能力削减平移指令。
 *
 * 轮速上限约束: |alpha·(vx,vy) + rot_i|² <= MAX_WHEEL_SPEED², 对四个轮子取最小的可行 alpha。
 * 其中 rot_i 必须与 SteeringCalculate() 用【同一套】自转分量符号, 否则这里会算出
 * 偏大或偏小的 alpha —— 偏小会让平移被白白削掉, 偏大则会让轮速实际超限。
 */
static void LimitTranslationForSpin(void)
{
    if (chassis_ctrl_cmd->chassis_mode != CHASSIS_ROTATE)
    {
        return;
    }

    const float translation_sq = Square(chassis_vx) + Square(chassis_vy);
    const float limit_sq = Square(MAX_WHEEL_SPEED);
    float translation_scale = 1.0f;

    if (translation_sq < 1.0e-6f)
    {
        return;
    }

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        const WheelVector_s rotation = {.x = kSteerRotX(i), .y = kSteerRotY(i)};

        // |alpha * translation + rotation_i|^2 <= MAX_WHEEL_SPEED^2
        const float b = 2.0f * (chassis_vx * rotation.x + chassis_vy * rotation.y);
        const float c = Square(rotation.x) + Square(rotation.y) - limit_sq;

        // 纯自转已经超过轮速能力，平移无法解决。
        if (c > 0.0f)
        {
            chassis_vx = 0.0f;
            chassis_vy = 0.0f;
            return;
        }

        const float discriminant = b * b - 4.0f * translation_sq * c;
        const float positive_root =
            (-b + sqrtf(fmaxf(discriminant, 0.0f))) / (2.0f * translation_sq);

        translation_scale = fminf(translation_scale, positive_root);
    }

    translation_scale = fmaxf(0.0f, translation_scale);
    chassis_vx *= translation_scale;
    chassis_vy *= translation_scale;
}
/**
 * @brief 在下发角度指令前加入 GM6020 机械零位偏移。
 *
 * 【为什么是 += 而不是 -=】
 *   角度环内部实际比较的是(见 dji_motor.c 的 DJIMotorControl + controller.c:159):
 *
 *       Err = pid_ref − pid_measure = (st/COEF − offset) − ecd
 *
 *   其中【测量侧 ecd 不含 offset】(motor 库不认识 rudder_offset, 见 grep 结果),
 *   所以 offset 只会出现一次, 不会被抵消。令 Err = 0 得稳态:
 *
 *       st = 0  =>  ecd = −offset     (旧写法)
 *       st = 0  =>  ecd = +offset     (本写法)
 *
 *   而零偏的定义是"舵轮指向正前方时的原始 ecd", 即期望稳态落在 ecd = offset。
 *   因此这里必须【加】, 写成减法会让舵轮跑到 −offset 那个姿态上(通常恰好差 180°)。
 *
 *   这正好解释了"认真标完零偏却完全不对": 标定本身没问题(失能时读的是机械正前方),
 *   是这份代码把 offset 加到了错误的方向, 导致任何零偏都不可能标正。
 */
static void ApplyRudderOffset(void)
{
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        rudder_angle_ref[i] += (float)chassis->rudder_offset[i] * ECD_ANGLE_COEF_DJI;
    }
}

/**
 * @brief 根据舵角跟随误差统一门控四个轮子的驱动速度。
 *
 * 8 度以内不衰减；误差从 8 度增加到 90 度时，公共门控系数从 1 平滑降到 0。
 * 使用公共系数可保持四轮扭矩比例，不额外改变底盘偏航力矩。
 */
static void ApplyRudderPriority(void)
{
    float common_scale = 1.0f;

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        float angle_error = fabsf(NormalizeAngle180(
            rudder_angle_ref[i] - chassis->rudder_motor[i]->measure.total_angle));
        float wheel_scale = 1.0f;

        if (angle_error > STEERING_NO_ATTENUATION_ANGLE)
        {
            if (angle_error >= 90.0f)
            {
                wheel_scale = 0.0f;
            }
            else
            {
                float normalized_error =
                    (angle_error - STEERING_NO_ATTENUATION_ANGLE) /
                    (90.0f - STEERING_NO_ATTENUATION_ANGLE);
                wheel_scale = arm_cos_f32(normalized_error * PI / 2.0f);
            }
        }

        if (wheel_scale < common_scale)
        {
            common_scale = wheel_scale;
        }
    }

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        wheel_speed_ref[i] *= common_scale;
    }
}

/**
 * @brief 用舵机实测角为舵角参考播种，并复位短弧翻转方向。
 *
 * 为什么需要这一步：rudder_angle_ref 的静态初值是 0，而电机使能后立即就会朝
 * rudder_angle_ref 闭环。若上电后第一次 ChassisTask() 时指令还在死区内(摇杆没动)，
 * SteeringCalculate() 不会刷新舵角参考，四个 GM6020 会一起朝 "0 度" 冲过去，
 * 等价于一次满舵动作 —— 未标定或装配后第一次上电时可能直接顶到机械限位。
 *
 * 因此在"舵电机使能 + 舵角参考尚未播种"的第一次任务里，把参考设成舵机当前
 * 实测角，即"原地保持不动"，之后交给正常解算刷新。
 */
static void SeedRudderReference(void)
{
    if (rudder_ref_seeded || chassis == NULL)
    {
        return;
    }

    if (!RudderCalibrationValid())
    {
        return; // 零偏未标定，舵电机本来就不会使能，不做任何事
    }

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        DJIMotorInstance *rudder = chassis->rudder_motor[i];

        // 还没收到过反馈时实测角没有意义：保持参考不动并推迟播种，下一周期再看。
        // 判据只能是"看门狗是否在线"：ecd 本身可以合法地等于 0，
        // 用 ecd==0 判断"没收到反馈"会让播种永远无法完成，参考值就一直是 0。
        if (!DaemonIsOnline(rudder->daemon))
        {
            return;
        }
    }

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        rudder_angle_ref[i] = chassis->rudder_motor[i]->measure.total_angle;
        wheel_direction[i] = 1;
    }

    rudder_ref_seeded = 1U;
}

/**
 * @brief GM6020 舵电机拟合功率估计。
 *
 * 输入电流单位为 A，角速度单位为 rad/s；只有正功率会计入底盘功率预算。
 */
static float RudderPowerForecast(float current_amp, float omega_rad_per_sec)
{
    const float k0 = 0.8130f;
    const float k1 = -0.0005f;
    const float k2 = 6.0021f;
    const float a = 1.3715f;

    return k0 * current_amp * omega_rad_per_sec + k1 * Square(omega_rad_per_sec) + k2 * Square(current_amp) + a;
}

/**
 * @brief 3508 轮电机拟合功率估计。
 */
static float WheelPowerForecast(float current_output, float speed_aps)
{
    float current_amp = CurrentOutputTo3508Amp(current_output);
    float omega = WheelSpeedToRadPerSec(speed_aps);

    return wheel_power_param.k0 + wheel_power_param.k1 * current_amp + wheel_power_param.k2 * omega +
           wheel_power_param.k3 * current_amp * omega + wheel_power_param.k4 * Square(current_amp) +
           wheel_power_param.k5 * Square(omega);
}

/**
 * @brief 根据四轮功率模型直接反解公共电流缩放系数。
 *
 * 令每个轮子的电流为 I_i * s，则四轮总功率可写成：
 *
 *     A * s^2 + B * s + C = available_power
 *
 * 公共缩放系数 s 作用于所有轮子，因此保持四轮电流/扭矩比例不变。
 */
static float SolveWheelCurrentScaleByPower(const float speed_aps[WHEEL_COUNT],
                                           const float current_output[WHEEL_COUNT],
                                           float available_power)
{
    float coefficient_a = 0.0f;
    float coefficient_b = 0.0f;
    float coefficient_c = 0.0f;

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        const float current_amp = CurrentOutputTo3508Amp(current_output[i]);
        const float omega = WheelSpeedToRadPerSec(speed_aps[i]);
        const float reference_power = WheelPowerForecast(current_output[i], speed_aps[i]);

        // 与 PowerControl 的总功率统计保持一致，不让负功率项抵消正功率项。
        if (reference_power <= 0.0f)
        {
            continue;
        }

        coefficient_a += wheel_power_param.k4 * Square(current_amp);
        coefficient_b += wheel_power_param.k1 * current_amp +
                         wheel_power_param.k3 * current_amp * omega;
        coefficient_c += wheel_power_param.k0 + wheel_power_param.k2 * omega +
                         wheel_power_param.k5 * Square(omega);
    }

    // 即使电流缩放到零，速度相关功率仍然超出预算，无法靠削减电流解决。
    if (coefficient_c >= available_power)
    {
        return 0.0f;
    }

    const float equation_c = coefficient_c - available_power;
    float current_scale;

    if (fabsf(coefficient_a) < 1.0e-6f)
    {
        // 退化为一次方程。
        if (coefficient_b <= 1.0e-6f)
        {
            return 1.0f;
        }
        current_scale = -equation_c / coefficient_b;
    }
    else
    {
        const float discriminant = coefficient_b * coefficient_b -
                                    4.0f * coefficient_a * equation_c;

        if (discriminant <= 0.0f)
        {
            return 0.0f;
        }

        // 取较大的根，得到不超功率时最大的可用电流比例。
        current_scale = (-coefficient_b + sqrtf(discriminant)) /
                         (2.0f * coefficient_a);
    }

    return fmaxf(0.0f, fminf(1.0f, current_scale));
}

/**
 * @brief 预测电机功率并限制轮电机电流输出。
 *
 * 舵电机功率优先计入预算；当预测轮电机功率超过 chassis_power_limit
 * 时，搜索一个公共电流缩放系数并作用于四个轮电机。
 */
static void PowerControl(void)
{
    float wheel_power[WHEEL_COUNT] = {0.0f};
    float wheel_current[WHEEL_COUNT] = {0.0f};
    float wheel_speed[WHEEL_COUNT] = {0.0f};
    float total_wheel_power = 0.0f;

    initial_rudder_total_power = 0.0f;
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        initial_rudder_give_power[i] =
            RudderPowerForecast(CurrentOutputTo6020Amp((float)chassis->rudder_motor[i]->measure.real_current),
                                WheelSpeedToRadPerSec(chassis->rudder_motor[i]->measure.speed_aps));

        if (initial_rudder_give_power[i] > 0.0f)
        {
            initial_rudder_total_power += initial_rudder_give_power[i];
        }
    }

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        wheel_speed[i] = chassis->wheel_motor[i]->measure.speed_aps;
        wheel_current[i] = chassis->wheel_motor[i]->motor_controller.final_output;
        wheel_power[i] = WheelPowerForecast(wheel_current[i], wheel_speed[i]);

        if (wheel_power[i] > 0.0f)
        {
            total_wheel_power += wheel_power[i];
        }
    }

    float available_power = (float)chassis_power_limit; // 新规则下不再需要控制舵电机功率 - initial_rudder_total_power;
    if (available_power < 0.0f)
    {
        available_power = 0.0f;
    }

    if (total_wheel_power > available_power && total_wheel_power > 0.0f)
    {
        float current_scale = SolveWheelCurrentScaleByPower(wheel_speed, wheel_current, available_power);

        for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
        {
            wheel_current[i] *= current_scale;
        }
    }

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        chassis->wheel_motor[i]->motor_controller.final_output = (int16_t)wheel_current[i];
    }
}

/**
 * @brief 写入轮电机/舵电机 PID 参考值，然后执行功率限制。
 */
static void LimitChassisOutput(void)
{
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
#if DEMO_RUDDER_ONLY_TEST
        // 只验舵向: 轮速参考强制为 0, 四个轮子只转向不滚动。
        // 用于在角度环极性还没确认时安全地单独验证舵角 —— 即使舵轮跑飞车也不会窜出去。
        DJIMotorSetPIDRef(chassis->wheel_motor[i], 0.0f);
#else
        DJIMotorSetPIDRef(chassis->wheel_motor[i], wheel_speed_ref[i]);
#endif
        DJIMotorSetPIDRef(chassis->rudder_motor[i], rudder_angle_ref[i]);
    }

    PowerControl();
}

/**
 * @brief 计算四个舵轮模块的逆运动学。
 *
 * 每个模块先由底盘 vx/vy/wz 得到目标速度向量, 再转换为舵角和轮速;
 * 必要时通过轮向反转把舵角约束到短弧路径。
 *
 * 【坐标系】+x = 右, +y = 前, +wz = 顺时针(俯视), 详见 kSteerPos 上方的说明。
 *
 * 【两个必须记住的结论, 它们直接对应实测期望值】
 *   1. 纯平移(自转=0): 四轮指向【同一个方向】, 该方向就是平移方向。
 *          +vy(前进) -> 四轮 st 全 =   0°(指向正前方, 即舵机零偏标定的位置)
 *          +vx(右移) -> 四轮 st 全 = +90°
 *          -vy(后退) -> 四轮 st 全 = 180°
 *          -vx(左移) -> 四轮 st 全 = -90°
 *      所以【静止不动时 st 必须接近 0°】—— 这也顺带验证了零偏是不是标在"正前方"。
 *   2. 自转(平移=0, +wz 顺时针):
 *          LF = 180°   LB = -90°   RB = 0°   RF = +90°
 *      四个方向同样与该轮位置处"绕中心顺时针"的切向一致。
 *
 * 【舵角为什么是 atan2(vx, vy) 而不是 atan2(vy, vx)】
 *   舵机 0° = 该轮指向【机械正前方】(零偏就是这么标定的, 见 chassis.h), 于是舵角
 *   必须从 +y 轴量起:
 *       st = 0°   -> 轮子指向 +y(前)
 *       st = +90° -> 轮子指向 +x(右)    =>  x 分量 = |v|·sin(st), y 分量 = |v|·cos(st)
 *                                             =>  st = atan2(vx, vy)
 *   用 atan2(vy, vx) 会让 0° 变成"指向 +x(右)", 四个轮子整体偏 90°, 表现为
 *   "推前进时车往右横移、推右移时车往后退"—— 这正是本次修正的问题。
 */
static void SteeringCalculate(void)
{
    LimitTranslationForSpin();

    /* 每轮目标速度矢量 = 平移分量 + 自转分量, 自转分量由 kSteerPos 唯一确定 */
    WheelVector_s wheel_vector[WHEEL_COUNT];
    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        wheel_vector[i].x = chassis_vx + kSteerRotX(i);
        wheel_vector[i].y = chassis_vy + kSteerRotY(i);
    }

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        wheel_speed_ref[i] = sqrtf(Square(wheel_vector[i].x) + Square(wheel_vector[i].y));
    }

    if (fabsf(chassis_ctrl_cmd->vx) > STEERING_CMD_DEADBAND ||
        fabsf(chassis_ctrl_cmd->vy) > STEERING_CMD_DEADBAND ||
        fabsf(chassis_ctrl_cmd->wz) > STEERING_CMD_DEADBAND)
    {
        for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
        {
            // 舵角从 +y(正前方) 量起, 逆时针为正 —— 与舵机零偏的标定位置一致
            rudder_angle_ref[i] = RAD_2_DEGREE * atan2f(wheel_vector[i].x, wheel_vector[i].y);
        }

        ApplyRudderOffset();

        for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
        {
            float current_angle = chassis->rudder_motor[i]->measure.total_angle;
            rudder_angle_ref[i] =
                AngleToOptimalAngle(rudder_angle_ref[i], current_angle, &wheel_direction[i]);
        }

        // 只要解算刷新过舵角，播种阶段就结束了。
        rudder_ref_seeded = 1U;
    }

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        wheel_speed_ref[i] *= (float)wheel_direction[i];
    }

    if (chassis_ctrl_cmd->chassis_mode != CHASSIS_ROTATE)
    {
        LimitWheelSpeed();
    }
}

/**
 * @brief 超级电容状态机和底盘功率预算选择。
 *
 * @note 未配置超级电容时不做状态机，直接把功率上限交给功率环：
 *       既不会去读 cap_msg(那需要超电回传，没有硬件时永远是 0V)，也不会
 *       把 max_power 压到 SAFETY_MODE 的 35W。
 */
static void UpdateSuperCapMode(void)
{
    uint16_t referee_power_limit = RefereePowerLimit();

    // 无超电时不需要状态机，只需要一个可用的功率预算：
    // 优先用裁判系统上报值，没有裁判系统则退回机器人设定值/组件兜底值。
    if (chassis->super_cap == NULL)
    {
        chassis_power_limit = (referee_power_limit > 0U) ? referee_power_limit : chassis_set_power_limit;
        return;
    }

    float cap_voltage = chassis->super_cap->cap_msg.cap_v;

    switch (chassis->super_cap_mode)
    {
    case SAFETY_MODE:
        if (cap_voltage > SUPERCAP_FULL_VOLTAGE)
        {
            chassis->super_cap_mode = PASSIVE_MODE;
        }
        chassis_power_limit = SUPERCAP_SAFE_POWER;
        break;

    case FORCED_CHARGING_MODE:
        if (cap_voltage < SUPERCAP_SAFETY_VOLTAGE)
        {
            chassis->super_cap_mode = SAFETY_MODE;
        }
        if (cap_voltage > SUPERCAP_FULL_VOLTAGE)
        {
            chassis->super_cap_mode = PASSIVE_MODE;
        }
        chassis_power_limit = PowerLimitMinus(referee_power_limit, 30U);
        break;

    case CHARGING_MODE:
        if (cap_voltage < SUPERCAP_FORCE_CHARGE_VOLTAGE)
        {
            chassis->super_cap_mode = FORCED_CHARGING_MODE;
        }
        if (cap_voltage > SUPERCAP_FULL_VOLTAGE)
        {
            chassis->super_cap_mode = PASSIVE_MODE;
        }
        chassis_power_limit = PowerLimitMinus(referee_power_limit, 20U);
        break;

    case PASSIVE_MODE:
        if (chassis_ctrl_cmd->SuperCapBoost & 1U)
        {
            chassis->super_cap_mode = ACTIVE_MODE;
        }
        if (cap_voltage < SUPERCAP_CHARGE_VOLTAGE)
        {
            chassis->super_cap_mode = CHARGING_MODE;
        }
        chassis_power_limit = (uint16_t)(0.9f * (float)referee_power_limit);
        break;

    case ACTIVE_MODE:
        if (cap_voltage < SUPERCAP_CHARGE_VOLTAGE)
        {
            chassis->super_cap_mode = CHARGING_MODE;
        }
        if (!(chassis_ctrl_cmd->SuperCapBoost & 1U))
        {
            chassis->super_cap_mode = PASSIVE_MODE;
        }
        chassis_power_limit = SUPERCAP_ACTIVE_POWER;
        break;

    default:
        chassis->super_cap_mode = SAFETY_MODE;
        chassis_power_limit = SUPERCAP_SAFE_POWER;
        break;
    }
}

/**
 * @brief 围绕目标角度加入带滞回的 yaw 跟随反馈。
 */
static void ApplyFollowControl(float target_angle)
{
    float angle_error = NormalizeAngle180(chassis_ctrl_cmd->offset_angle - target_angle);

    if (fabsf(angle_error) > FOLLOW_START_DEADBAND)
    {
        follow_enabled = 1U;
    }
    if (fabsf(angle_error) < FOLLOW_STOP_DEADBAND)
    {
        follow_enabled = 0U;
    }

    if (follow_enabled)
    {
        chassis_ctrl_cmd->wz += PIDCalculate(&follow_pid, angle_error, 0.0f);
    }
}

/**
 * @brief 在运动学解算前处理高层底盘模式逻辑。
 */
static void ApplyChassisMode(void)
{
    switch (chassis_ctrl_cmd->chassis_mode)
    {
    case CHASSIS_FOLLOW:
        ApplyFollowControl(0.0f);
        break;

    case CHASSIS_FOLLOW_DIAGONAL:
        ApplyFollowControl(45.0f);
        break;

    case CHASSIS_ROTATE:
        chassis_ctrl_cmd->offset_angle -= 0.001f * chassis_ctrl_cmd->wz;
        break;

    case CHASSIS_FREE:
        // 纯手动：不自转跟随、不累加 offset_angle，平移与自转全部由上层摇杆给定。
        break;

    default:
        break;
    }
}

/**
 * @brief 把【底盘系指令】按底盘相对世界系的偏角旋转, 得到世界系下的平移指令。
 *
 * 链路是: 摇杆 -> chassis_ctrl_cmd(vx, vy) [底盘系] --本函数--> (chassis_vx, chassis_vy)
 *         [世界系] -> SteeringCalculate() 解算舵角轮速。
 *
 * offset_angle 的定义: 世界系 x 轴(车头朝前时指向右) 相对底盘系 x 轴 的转角, 逆时针为正,
 * 即底盘相对世界系转过了 -offset_angle。因此底盘系 -> 世界系 需要左乘 R(+offset_angle):
 *
 *     [chassis_vx]   [ cosθ  -sinθ ] [vx]
 *     [chassis_vy] = [ sinθ   cosθ ] [vy]
 *
 * @note offset_angle 恒为 0 时(本 demo)本函数必须严格退化为【原样透传】。
 *       之前这里用 R(-offset_angle) 并且额外给两个分量都加了负号, θ=0 时变成
 *       v -> (-vx, -vy), 也就是【把摇杆指令整体反向 180°】, 前进变后退、
 *       右移变左移、自转方向也反 —— 这是之前"方向全错"的直接原因之一。
 */
static void TransformCommandToChassisFrame(void)
{
    float cos_theta = arm_cos_f32(chassis_ctrl_cmd->offset_angle * DEGREE_2_RAD);
    float sin_theta = arm_sin_f32(chassis_ctrl_cmd->offset_angle * DEGREE_2_RAD);

    chassis_vx = chassis_ctrl_cmd->vx * cos_theta - chassis_ctrl_cmd->vy * sin_theta;
    chassis_vy = chassis_ctrl_cmd->vx * sin_theta + chassis_ctrl_cmd->vy * cos_theta;
}

/**
 * @brief 将裁判系统功率状态转发给超电模块。
 *
 * @note 未配置超电时直接返回，不产生任何发送动作。
 */
static void SendSuperCapCommand(void)
{
    if (chassis->super_cap == NULL)
    {
        return;
    }

    SuperCapSendMessage(chassis->super_cap, ClampPowerForSuperCap(RefereePowerLimit()),
                        referee_data->PowerHeatData.buffer_energy,
                        referee_data->GameRobotState.power_management_chassis_output);
}

/**
 * @brief 分配并连接舵轮底盘运行时实例。
 *
 * 该函数保存共享的裁判系统指针，初始化可选的超电模块和四组轮/舵电机，
 * 并设置 ChassisTask() 使用的模块级指针。初始化配置仍由调用方持有；
 * 返回的实例由 zmalloc 分配，并作为本模块的单例保存。
 *
 * @return 初始化成功时返回 ChassisInstance 指针；内存分配失败时返回 NULL。
 */
ChassisInstance *ChassisInit(Chassis_Init_Config_s *chassis_init_config)
{
    ChassisInstance *chassis_instance = (ChassisInstance *)zmalloc(sizeof(ChassisInstance));
    if (chassis_instance == NULL)
    {
        return NULL;
    }

    wheel_power_param = chassis_init_config->chassis_param.power_param;
    referee_data = GetReferee();
    PIDInit(&follow_pid, &chassis_init_config->follow_pid);

    // 超级电容是可选的：没有 can_handle 就不注册，避免拿空句柄去初始化 CAN
    // (SuperCapInit 内部会 CANRegister，首次注册还会触发 CANServiceInit，
    //  传 NULL 会让 HAL_CAN_Start 直接解引用空指针)。
    if (chassis_init_config->super_cap_config.can_config.can_handle != NULL)
    {
        chassis_instance->super_cap = SuperCapInit(&chassis_init_config->super_cap_config);
    }
    else
    {
        chassis_instance->super_cap = NULL;
    }
    chassis_instance->super_cap_mode = SAFETY_MODE;

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        ConfigureWheelMotor(&chassis_init_config->wheel_motor_config[i]);
        chassis_instance->wheel_motor[i] = DJIMotorInit(&chassis_init_config->wheel_motor_config[i]);
    }

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        ConfigureRudderMotor(&chassis_init_config->rudder_motor_config[i]);
        chassis_instance->rudder_motor[i] = DJIMotorInit(&chassis_init_config->rudder_motor_config[i]);
        chassis_instance->rudder_offset[i] = chassis_init_config->chassis_param.rudder_motor_offset[i];
    }

    chassis = chassis_instance;
    chassis_ctrl_cmd = &chassis->chassis_ctrl_cmd;
    chassis_set_power_limit = CHASSIS_DEFAULT_NO_REFEREE_POWER;
    chassis_power_limit = chassis_set_power_limit;
    rudder_ref_seeded = 0U;

    return chassis_instance;
}

/**
 * @brief 舵轮底盘周期控制任务。
 *
 * 该任务需要在 ChassisInit() 之后运行。每个周期会更新超电功率模式、
 * 应用底盘模式、把云台系指令转换到底盘系、执行舵轮逆运动学解算，
 * 最后写入电机参考值并进行功率限制。
 */
void ChassisTask(void)
{
    if (chassis == NULL || chassis_ctrl_cmd == NULL)
    {
        return;
    }

    UpdateSuperCapMode();
    EnableChassisMotors(chassis_ctrl_cmd->chassis_mode != CHASSIS_POWER_OFF);
    SeedRudderReference();
    ApplyChassisMode();
    TransformCommandToChassisFrame();

    if (chassis_ctrl_cmd->chassis_mode != CHASSIS_POWER_OFF)
    {
        SteeringCalculate();
        ForwardKinematicCal();
        ApplyRudderPriority();
    }

    SendSuperCapCommand();
    LimitChassisOutput();
}

/* ==========================================================================================
 *  只读导出 —— 供 robot 层观测解算中间量与安全检查, 不参与控制逻辑。
 *
 *  为什么需要它: rudder_angle_ref / wheel_speed_ref / chassis_vx / chassis_vy 都是本文件的
 *  static, 外部读不到。排查"指令对了但轮子动作不对"时, 必须能看到【解算出来的舵角目标】
 *  本身, 才能区分是"解算算错了"还是"电机没执行对"。
 * ========================================================================================== */
void ChassisGetDebugState(Chassis_Debug_State_s *out)
{
    if (out == NULL)
    {
        return;
    }

    out->chassis_vx = chassis_vx;
    out->chassis_vy = chassis_vy;

    for (uint8_t i = 0; i < WHEEL_COUNT; ++i)
    {
        out->st[i] = rudder_angle_ref[i];
        out->vt[i] = wheel_speed_ref[i];
        out->dir[i] = wheel_direction[i];

        // 【原始】编码器角(度, 0~360): 未减零偏、不受任何 reverse flag 影响。
        // 标定零偏时直接抄这个; 零偏 = 舵轮指向正前方时的本值。
        out->ecd_deg[i] = chassis->rudder_motor[i]->measure.angle_single_round;

        /**
         * 实测舵角(度) = "该轮现在指向哪", 直接用 原始ecd - 零偏 得到。
         *
         * 为什么可以直接相减: 零偏的定义就是【指向正前方时的原始 ecd】, 所以这个差值
         * 天然就是"相对正前方转过了多少度", 与两个 reverse flag 都无关
         * (flag 只影响电机库内部把反馈取不取负, 不改变"这个数代表哪个物理姿态")。
         *
         * 用途: 上电静止时它应接近 0(= 指向正前方); 跟随过程中可与 st 对照看舵机
         *       有没有跟上。若它稳定停在某个固定偏角, 说明零偏标错了。
         */
        out->cur[i] = NormalizeAngle180(chassis->rudder_motor[i]->measure.angle_single_round -
                                        (float)chassis->rudder_offset[i] * ECD_ANGLE_COEF_DJI);
    }
}

uint8_t ChassisIsRudderReady(void)
{
    return RudderCalibrationValid();
}

void ChassisSetPowerLimit(uint16_t power_limit)
{
    chassis_set_power_limit = power_limit;
}

uint16_t ChassisGetPowerLimit(void)
{
    return chassis_power_limit;
}
