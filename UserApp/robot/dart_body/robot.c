
#include "robot.h"

#include <math.h>

#include "bsp_dwt.h"
#include "bsp_log.h"
#include "controller.h"
#include "ins_task.h"
#include "main.h"
#include "tim.h"
#include "user_lib.h"
#include "wing.h"

static RobotInstance *robot = NULL;
static float l_z_r[SERVO_CH_NUM][3] = SERVO_LZR_TABLE;
static INS_t *ins = NULL; // 板载 BMI088 解算结果

static int wing_dirs[4] = WING_DIRS;

//本文件统一按 [0]=yaw [1]=pitch [2]=roll 组织 此处写为全局变量只是因为在测试时方便改动数值 后续test开头的估计会全部删除掉
volatile float test_alpha[3] = {0.0f, 0.0f, 0.0f};
//台架调试: 0 = 直接给攻角(test_alpha), 1 = 走角速度环。
volatile uint8_t test_mode = 0;
// 角速度环的指令输入(deg/s), 依旧[0]=yaw [1]=pitch [2]=roll。给 0 就是"增稳"模式。
volatile float test_rate[3] = {0.0f, 0.0f, 0.0f};

/* 飞行状态机
 * 照抄参考 func/FSM_Config.c 的两态机与判据。此阶段暂时用不到*/
typedef enum
{
    STATIC = 0,
    SHOOTING = 1
} FSM_ShootingStateTypeDef;

typedef struct
{
    FSM_ShootingStateTypeDef shooting_state;
    FSM_ShootingStateTypeDef last_shooting_state;
    uint8_t is_weightlessness;
    uint32_t shooting_time_start;
    uint32_t shooting_state_change_time;
} FSM_ShootingConfigTypeDef;

static FSM_ShootingConfigTypeDef fsm_config;

static void ShootingFsmInit(void)
{
    uint32_t now = DWT_GetTimeline_ms();
    fsm_config.shooting_state = STATIC;
    fsm_config.last_shooting_state = STATIC;
    fsm_config.is_weightlessness = 0;
    fsm_config.shooting_time_start = now - (uint32_t)STATIC_TO_SHOOTING_TIME_DECIDE;
    fsm_config.shooting_state_change_time = now - (uint32_t)STATIC_TO_SHOOTING_TIME_DECIDE;
}

static void ShootingStateChange(void)
{
    uint32_t now = DWT_GetTimeline_ms();

    if (fsm_config.shooting_state == STATIC)
    {
        if (now - fsm_config.shooting_state_change_time <= (uint32_t)STATIC_TO_SHOOTING_TIME_DECIDE)
        {
            fsm_config.shooting_state_change_time = now;
        }
        else
        {
            fsm_config.shooting_state = SHOOTING;
            fsm_config.shooting_time_start = now;
            fsm_config.shooting_state_change_time = now;
            LOGINFO("[dart] state -> SHOOTING");
        }
    }
    else
    {
        if (now - fsm_config.shooting_state_change_time <= (uint32_t)SHOOTING_TO_STATIC_TIME_DECIDE)
        {
            fsm_config.shooting_state_change_time = now;
        }
        else
        {
            fsm_config.shooting_state = STATIC;
            fsm_config.shooting_state_change_time = now;
            LOGINFO("[dart] state -> STATIC");
        }
    }
}

static void ShootingFsmUpdate(void)
{
    if (ins == NULL)
        return;
    float a2 = ins->Accel[0] * ins->Accel[0] + ins->Accel[1] * ins->Accel[1] + ins->Accel[2] * ins->Accel[2];

    fsm_config.last_shooting_state = fsm_config.shooting_state;

    if (fsm_config.shooting_state == SHOOTING)
    {
        if (a2 > SHOOTING_TO_STATIC_ACCELERATION_THRESHOLD * SHOOTING_TO_STATIC_ACCELERATION_THRESHOLD)
        {
            ShootingStateChange();
        }
        else if (a2 < SHOOTING_WEIGHTLESSNESS_ACCELERATION_THRESHOLD * SHOOTING_WEIGHTLESSNESS_ACCELERATION_THRESHOLD)
        {
            fsm_config.is_weightlessness = 1;
        }
        else
        {
            fsm_config.is_weightlessness = 0;
        }
    }
    else
    {
        if (a2 > SHOOTING_ACCELERATION_THRESHOLD * SHOOTING_ACCELERATION_THRESHOLD)
        {
            ShootingStateChange();
        }
        else if (a2 < SHOOTING_WEIGHTLESSNESS_ACCELERATION_THRESHOLD * SHOOTING_WEIGHTLESSNESS_ACCELERATION_THRESHOLD)
        {
            fsm_config.is_weightlessness = 1;
        }
        else
        {
            fsm_config.is_weightlessness = 0;
        }
    }
}


/* 角速度环
 * 输入: rate_cmd_dps(制导层写入, 现在恒 0), 反馈 ins->Gyro(rad/s)
 * 输出: alpha_deg(舵面攻角), 交给 WingSetAlpha*/
#define RAD2DEG 57.29578f /* rad/s -> deg/s */

static PIDInstance rate_pid[3]; // [0]=yaw [1]=pitch [2]=roll
static float rate_filt[3];      // 角速度低通滤波后的值(deg/s)
static float rate_cmd_dps[3];   //角速度指令, 由制导层写，此处只是接口，依旧用不到
static float alpha_deg[3];      // 攻角输出
//初始化角速度环
static void RateLoopInit(void)
{
    PID_Init_Config_s cfg = {
        .Kp = RATE_KP,
        .Ki = RATE_KI,
        .Kd = RATE_KD,
        .MaxOut = ALPHA_MAX,
        .IntegralLimit = RATE_I_LIMIT,
        .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit,
    };
    for (int i = 0; i < 3; i++)
    {
        PIDInit(&rate_pid[i], &cfg);
        rate_cmd_dps[i] = 0.0f;
        alpha_deg[i] = 0.0f;
        rate_filt[i] = 0.0f;
    }
}

static void RateLoopUpdate(void)
{
    if (ins == NULL)
        return;
    //一个简单的线性映射，之前用的北理atan映射在台架测试下对轻微的扰动反应过于明显 ，此处只是简单用线性映射确认对于角度变化舵面有正确的反应（其实正确吗 还是得要靠模拟吧）
    float rate_dps[3];
    rate_dps[0] = ins->Gyro[2] * RAD2DEG; // yaw
    rate_dps[1] = ins->Gyro[0] * RAD2DEG; // pitch
    rate_dps[2] = ins->Gyro[1] * RAD2DEG; // roll

    for (int i = 0; i < 3; i++)
    {
        //一个去尖峰和低通滤波 依旧是台架测试不让舵面乱抖做的
        if (fabsf(rate_dps[i]) > RATE_SPIKE_DPS)//去尖峰
        {
            rate_dps[i] = rate_filt[i];
        }
        else//先去尖峰再低通滤波
        {
            rate_filt[i] += RATE_LPF_ALPHA * (rate_dps[i] - rate_filt[i]);
            rate_dps[i] = rate_filt[i];
        }
        if (rate_cmd_dps[i] > RATE_MAX_DPS)
            rate_cmd_dps[i] = RATE_MAX_DPS;
        if (rate_cmd_dps[i] < -RATE_MAX_DPS)
            rate_cmd_dps[i] = -RATE_MAX_DPS;
    }

    alpha_deg[0] = PIDCalculate(&rate_pid[0], rate_dps[0], rate_cmd_dps[0]); // yaw
    alpha_deg[1] = PIDCalculate(&rate_pid[1], rate_dps[1], rate_cmd_dps[1]); // pitch
    alpha_deg[2] = PIDCalculate(&rate_pid[2], rate_dps[2], rate_cmd_dps[2]); // roll
}

void RobotInit(void)
{
    robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));

    // 4 路舵机注册 + 混控初始化都收在 WingInit 里了
    WingInit(&robot->wing, l_z_r, wing_dirs);

    ShootingFsmInit();
    RateLoopInit();
    ins = INS_Init(&imu_config);
}

void RobotTask(void)
{
    if (robot == NULL)
        return;

    if (ins == NULL)
        return;

    ShootingFsmUpdate();

    if (test_mode == 0)
    {
        // 模式 0: 直接给攻角, 用来验方向/行程
        WingSetAlpha(&robot->wing, test_alpha[0], test_alpha[1], test_alpha[2]);
    }
    else
    {
        //模式 1: 角速度环。test_rate 全 0 时就是抵抗转动
        for (int i = 0; i < 3; i++)
            rate_cmd_dps[i] = test_rate[i];

        RateLoopUpdate();
        WingSetAlpha(&robot->wing, alpha_deg[0], alpha_deg[1], alpha_deg[2]); // yaw, pitch, roll
    }
}
