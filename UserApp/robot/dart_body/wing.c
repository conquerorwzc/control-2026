#include "wing.h"

#include <math.h>

#include "tim.h"

// 这里是舵机执行层
static uint32_t kServoChannel[SERVO_CH_NUM] = SERVO_CH_TABLE;
// 角度转占空比
static float DartAngleToDuty(float angle)
{
    if (angle < 0.0f)
        angle = 0.0f;
    if (angle > 180.0f)
        angle = 180.0f;
    return SERVO_DUTY_MIN + (SERVO_DUTY_MAX - SERVO_DUTY_MIN) * (angle / 180.0f);
}
// 给角度
static int DartServoSetAngle(servo_typedef *servo, float angle)
{
    if (angle > 90.0f)
    {
        angle = servo->zero + (servo->right - servo->zero) * (angle - 90.0f) / 90.0f;
    }
    else
    {
        angle = servo->zero - (servo->left - servo->zero) * (angle - 90.0f) / 90.0f;
    }
    PWMSetDutyRatio(servo->pwm, DartAngleToDuty(angle));
    servo->angle = angle;
    return 0;
}

// 舵机初始化
static int DartServoEnable(servo_typedef *servo);
static int DartServoInit(servo_typedef *servo, int channel, float l_z_r[3])
{
    servo->left = l_z_r[0];
    servo->zero = l_z_r[1];
    servo->right = l_z_r[2];
    servo->angle = servo->zero;

    PWM_Init_Config_s cfg = {
        .htim = SERVO_PWM_TIM,
        .channel = kServoChannel[channel],
        .period = SERVO_PERIOD,
        .dutyratio = DartAngleToDuty(servo->zero),
        .callback = NULL,
        .id = servo,
    };
    servo->pwm = PWMRegister(&cfg);
    DartServoEnable(servo);
    return DartServoSetAngle(servo, 90.0f);
}

// 使能and失能
static int DartServoEnable(servo_typedef *servo)
{
    PWMStart(servo->pwm);
    return 0;
}
static int DartServoDisable(servo_typedef *servo)//好吧这个失能好像没用到 先留着吧
{
    PWMStop(servo->pwm);
    return 0;
}

// 混控部分

/* 攻角 -> 舵面偏角。
 *目前用了两套映射 用 WING_DECODE_LINEAR 切换 默认1为线性 台架调试时抖动没那么明显
 *切0时换成参考代码的 atan 曲线: servo = atan(alpha) * 45 / atan(ALPHA_MAX)
 *这条曲线在 0 附近斜率约 30 (1°攻角 -> 24°舵面), 是线性的 6.7 倍。
 *台架测试时表现为一挪就抖 后续再说
 */
#define WING_DECODE_LINEAR 1

static float WingF(float alpha)
{
    if (alpha > ALPHA_MAX)
        alpha = ALPHA_MAX;
    if (alpha < -ALPHA_MAX)
        alpha = -ALPHA_MAX;
#if WING_DECODE_LINEAR
    return alpha * (45.0f / ALPHA_MAX);
#else
    return atanf(alpha) * (45.0f / atanf(ALPHA_MAX));
#endif
}

void WingInit(wing_typedef *wing, float l_z_r[4][3],int dirs[4])
{
    for (int i = 0; i < 4; i++)
        wing->dirs[i] = dirs[i];
    // 4 路舵机初始化
    for (int i = 0; i < SERVO_CH_NUM; i++)
    {
        DartServoInit(&wing->servo[i], i, l_z_r[i]);
        DartServoSetAngle(&wing->servo[i], 90.0f);
    }

    wing->alpha_yaw = 0.0f;
    wing->alpha_pitch = 0.0f;
    wing->alpha_roll = 0.0f;
    for (int i = 0; i < 4; i++)
        wing->angle[i] = 0.0f;
}

void WingSetAlpha(wing_typedef *wing, float alpha_yaw, float alpha_pitch, float alpha_roll)
{
    float a_yaw = alpha_yaw;
    float a_pitch = -alpha_pitch;
    float a_roll = -alpha_roll;

    if (a_yaw > ALPHA_MAX)
        a_yaw = ALPHA_MAX;
    else if (a_yaw < -ALPHA_MAX)
        a_yaw = -ALPHA_MAX;
    if (a_pitch > ALPHA_MAX)
        a_pitch = ALPHA_MAX;
    else if (a_pitch < -ALPHA_MAX)
        a_pitch = -ALPHA_MAX;
    if (a_roll > ALPHA_MAX)
        a_roll = ALPHA_MAX;
    else if (a_roll < -ALPHA_MAX)
        a_roll = -ALPHA_MAX;

    wing->alpha_yaw = a_yaw;
    wing->alpha_pitch = a_pitch;
    wing->alpha_roll = a_roll;

    float angle_yaw = WingF(a_yaw);
    float angle_pitch = WingF(a_pitch);
    float angle_roll = WingF(a_roll);

    // 参考 func/wing.c:88-91 的混控矩阵, 这里直接原样照抄了
    float angles[4];
    angles[0] = angle_pitch + angle_yaw + angle_roll;
    angles[1] = angle_pitch - angle_yaw + angle_roll;
    angles[2] = -angle_pitch + angle_yaw + angle_roll;
    angles[3] = -angle_pitch - angle_yaw + angle_roll;

    WingSetAngle(wing, angles);
}

void WingSetAngle(wing_typedef *wing, float *angle)
{
    for (int i = 0; i < 4; i++)
    {
        if (angle[i] > 45.0f)
            angle[i] = 45.0f;
        if (angle[i] < -45.0f)
            angle[i] = -45.0f;

        wing->angle[i] = angle[i];

        float servo_deg = (wing->dirs[i] == 1) ? (90.0f + angle[i]) : (90.0f - angle[i]);
        DartServoSetAngle(&wing->servo[i], servo_deg);
    }
}
