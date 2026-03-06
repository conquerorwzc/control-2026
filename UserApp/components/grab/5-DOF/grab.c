//
// Created by ROG on 2025/11/17.
//
/* Private includes ----------------------------------------------------------*/
#include "grab.h"
#include "daemon.h"
#include "user_lib.h"

/* Private macro -------------------------------------------------------------*/
#define PULLEY_GEAR_RATIO 2.0f          // 带轮传动比
#define BEVEL_GEAR_RATIO 1.6667f        // 锥齿轮传动比 5:3
#define PLANAR_GEAR_RATIO 1.571428f     // 平面齿轮传动比 11:7
#define MOTOR2006_REDUCTION_RATIO 36.0f // 2006 ecd减速比36

#define DM_HOMING_TOLERANCE      2.0f   // DM大臂物理归零的角度容差 (度)
#define DM_CALI_MAX_TICKS        5000   // 阶段一：大臂归零最大允许时间 5 秒 (假设1ms调度)
#define WRIST_CALI_MAX_TICKS     6000   // 阶段二：腕部抬头堵转最大允许时间 6 秒
#define WRIST_CALI_SPEED         0.05f  // 腕部抬升速度
#define WRIST_CALI_CHECK_TICKS   500    // 堵转检测时间
#define WRIST_CALI_TOLERANCE     2.0f   // 堵转容差度数
#define WRIST_CALI_STALL_CURRENT 800   // 堵转电流阈值


/* Private variables ---------------------------------------------------------*/
static GrabInstance *grab;
static Grab_Ctrl_Cmd_s *grab_ctrl_cmd;
static float total_angle_init_L = 0;
static float total_angle_init_R = 0;
static float total_angle_init_M = 0;
static float total_angle_init_Video_forward = 0;
static float total_angle_init_Video_pitch = 0;
float a[5] = {0, 0, 0, 0, 0}; // 测试用电机目标位置

/* Private function prototypes -----------------------------------------------*/
GrabInstance *GrabInit(Grab_Init_Config_s *Grab_init_config); // 机械臂初始化，返回一个机械臂示例指针
void GrabTask();                                              // 机械臂任务函数
static void GrabCmdTask();                                    // 机械臂控制命令处理函数
static void MotorTask();                                      // 电机任务函数
static void Grab_Position_Calculate(GrabInstance *grab);      // 计算电机目标位置
static void GrabCalibrationTask(void);                        // 机械臂两段式安全标定任务

/* Private user code ---------------------------------------------------------*/
/**
 * @brief 初始化机械臂
 */
GrabInstance *GrabInit(Grab_Init_Config_s *Grab_init_config)
{
    GrabInstance *grab_instance = (GrabInstance *)zmalloc(sizeof(GrabInstance));
    grab_instance->actuator = (ActuatorInstance *)zmalloc(sizeof(ActuatorInstance));
    grab_instance->arm = (ArmInstance *)zmalloc(sizeof(ArmInstance));
    grab_instance->video = (VideoInstance *)zmalloc(sizeof(VideoInstance));

    grab_instance->actuator->grab_djimotor[0] = DJIMotorInit(&Grab_init_config->Grab_motor_config[3]);
    grab_instance->actuator->grab_djimotor[1] = DJIMotorInit(&Grab_init_config->Grab_motor_config[4]);
    grab_instance->actuator->grab_djimotor[2] = DJIMotorInit(&Grab_init_config->Grab_motor_config[8]);

    // 在没有上电的情况下先不发使能帧给dm电机，即不初始化
    grab_instance->actuator->grab_dmmotor[0] = DMMotorInit(&Grab_init_config->Grab_motor_config[5]); // v2
    grab_instance->arm->grab_dmmotor[0] = DMMotorInit(&Grab_init_config->Grab_motor_config[0]);      // v3
    grab_instance->arm->grab_dmmotor[1] = DMMotorInit(&Grab_init_config->Grab_motor_config[1]);      // v4
    grab_instance->arm->grab_dmmotor[2] = DMMotorInit(&Grab_init_config->Grab_motor_config[2]);      // v4

    // grab_instance->video->grab_djimotor[0] =
    // DJIMotorInit(&Grab_init_config->Grab_motor_config[6]);
    // grab_instance->video->grab_djimotor[1] =
    // DJIMotorInit(&Grab_init_config->Grab_motor_config[7]);

    // 先赋值grab指针，再访问grab_instance中的成员
    grab = grab_instance;
    grab_ctrl_cmd = &grab->grab_ctrl_cmd;

    // 初始化电机初始角度，必须要发生在grab的赋值之后
    osDelay(10);
    total_angle_init_L = grab->actuator->grab_djimotor[1]->measure.total_angle;
    total_angle_init_R = grab->actuator->grab_djimotor[0]->measure.total_angle;
    total_angle_init_M = grab_instance->actuator->grab_djimotor[2]->measure.total_angle;

    // total_angle_init_Video_pitch = grab->video->grab_djimotor[1]->measure.total_angle;
    if (Grab_init_config->Grab_cali_mode == GRAB_CALI_MODE)
    {
        DMMotorCaliEncoder(grab->arm->grab_dmmotor[0]);
        DMMotorCaliEncoder(grab->arm->grab_dmmotor[1]);
        DMMotorCaliEncoder(grab->arm->grab_dmmotor[2]);
        DMMotorCaliEncoder(grab->actuator->grab_dmmotor[0]);
    }
    return grab_instance;
}

/**
 * @brief 机械臂任务函数
 */
void GrabTask()
{
    grab_ctrl_cmd = &grab->grab_ctrl_cmd;
    GrabCmdTask();

    if (grab->actuator->wrist_cali.state != CALI_SUCCESS) {
        GrabCalibrationTask();
    } else {
        Grab_Position_Calculate(grab);
    }


    MotorTask();
}

static void GrabCmdTask()
{
    grab->arm->base_joint = grab_ctrl_cmd->base_joint;
    grab->arm->elbow_roll = grab_ctrl_cmd->elbow_roll;
    grab->arm->elbow_pitch = grab_ctrl_cmd->elbow_pitch;
    grab->actuator->wrist_pitch = grab_ctrl_cmd->wrist_pitch;
    grab->actuator->wrist_roll = grab_ctrl_cmd->wrist_roll;
    grab->actuator->torque = grab_ctrl_cmd->torque;

    // grab->video->video_forward= grab_ctrl_cmd->video_forward;
    // grab->video->video_pitch= grab_ctrl_cmd->video_pitch;
}

static void MotorTask()
{
    if (grab_ctrl_cmd->grab_mode == GRAB_POWER_OFF)
    {
        DMMotorStop(grab->arm->grab_dmmotor[0]);
        DMMotorStop(grab->arm->grab_dmmotor[1]);
        DMMotorStop(grab->arm->grab_dmmotor[2]);

        DJIMotorStop(grab->actuator->grab_djimotor[0]);
        DJIMotorStop(grab->actuator->grab_djimotor[1]);
        DJIMotorStop(grab->actuator->grab_djimotor[2]);
        DMMotorStop(grab->actuator->grab_dmmotor[0]);

        // DJIMotorStop(grab->video->grab_djimotor[0]);
        // DJIMotorStop(grab->video->grab_djimotor[1]);
    }
    else
    {
        // 循环处理所有DMMotor，只对在线的电机进行使能和PID计算
        for (int i = 0; i < 3; i++)
        {
            if (DaemonIsOnline(grab->arm->grab_dmmotor[i]->daemon))
            {
                DMMotorEnable(grab->arm->grab_dmmotor[i]);
                switch (i)
                {
                case 0:
                    DMMotorSetPIDRef(grab->arm->grab_dmmotor[i], grab->arm->base_joint * DEGREE_2_RAD);
                    break;
                case 1:
                    DMMotorSetPIDRef(grab->arm->grab_dmmotor[i], grab->arm->elbow_roll * DEGREE_2_RAD);
                    break;
                case 2:
                    DMMotorSetPIDRef(grab->arm->grab_dmmotor[i], grab->arm->elbow_pitch * DEGREE_2_RAD);
                    break;
                }
            }
        }

        // 循环处理所有DJIMotor（actuator部分）
        for (int i = 0; i < 3; i++)
        {
            if (DaemonIsOnline(grab->actuator->grab_djimotor[i]->daemon))
            {
                DJIMotorEnable(grab->actuator->grab_djimotor[i]);
                switch (i)
                {
                case 0:
                    DJIMotorSetPIDRef(grab->actuator->grab_djimotor[i], grab->actuator->R_target);
                    break;
                case 1:
                    DJIMotorSetPIDRef(grab->actuator->grab_djimotor[i], grab->actuator->L_target);
                    break;
                case 2:
                    DJIMotorSetPIDRef(grab->actuator->grab_djimotor[i], grab->actuator->M_target);
                    break;
                }
            }
        }

        // 处理actuator的DMMotor
        if (DaemonIsOnline(grab->actuator->grab_dmmotor[0]->daemon))
        {
            DMMotorEnable(grab->actuator->grab_dmmotor[0]);
            DMMotorSetRef(grab->actuator->grab_dmmotor[0], grab->actuator->T_target);
        }

        // // 循环处理所有DJIMotor（Video部分）
        // for (int i = 0; i < 2; i++) {
        //     if (DaemonIsOnline(grab->video->grab_djimotor[i]->daemon)) {
        //         DJIMotorEnable(grab->video->grab_djimotor[i]);
        //         switch (i) {
        //             case 0:
        //                 DJIMotorSetPIDRef(grab->video->grab_djimotor[i], grab->video->F_target);
        //                 break;
        //             case 1:
        //                 DJIMotorSetPIDRef(grab->video->grab_djimotor[i], grab->video->P_target);
        //                 break;
        //         }
        //     }
        // }
    }
}

/**
 * @brief 根据pitch和roll角度解算出电机的转动角度
 * @param arm 机械臂结构体指针
 */
static void Grab_Position_Calculate(GrabInstance *grab)
{
    /**
     * L = +pitch (同向驱动)
     * R = +pitch (同向驱动)
     * M = +roll  (独立电机驱动平面齿轮)
     */

    grab->actuator->R_target =
        total_angle_init_R + grab->actuator->wrist_pitch * MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;

    grab->actuator->L_target =
        total_angle_init_L + grab->actuator->wrist_pitch * MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;

    grab->actuator->M_target =
        total_angle_init_M + grab->actuator->wrist_roll * MOTOR2006_REDUCTION_RATIO * PLANAR_GEAR_RATIO;

    // grab->video->F_target = total_angle_init_Video_forward + grab->video->video_forward * MOTOR2006_REDUCTION_RATIO;
    // grab->video->P_target = total_angle_init_Video_pitch + grab->video->video_pitch;

    grab->actuator->T_target = grab->actuator->torque;
}

/**
 * @brief 两段式安全标定任务 (底层限速物理归零 -> 腕部 Pitch 抬头)
 */
static void GrabCalibrationTask(void)
{
    static float cali_pitch = 0.0f;
    static float last_r_angle = 0, last_l_angle = 0;
    static uint16_t block_cnt = 0;
    static uint32_t timeout_cnt = 0;
    static uint8_t first_run = 1;

    static GrabCaliStage_e cali_stage = CALI_STAGE_DM_WAIT_ZERO;

    //  急停/未使能感知与记忆擦除
    if (grab_ctrl_cmd->grab_mode == GRAB_POWER_OFF)
    {
        first_run = 1;
        block_cnt = 0;
        timeout_cnt = 0;
        cali_stage = CALI_STAGE_DM_WAIT_ZERO;
        return;
    }

    // 2. 首次运行初始化 (带离线保护)
    if (first_run)
    {
        // 🚨 核心保护：必须等待3个 2006 电机都上线！否则读到的角度是0，直接疯转。
        if (!DaemonIsOnline(grab->actuator->grab_djimotor[0]->daemon) ||
            !DaemonIsOnline(grab->actuator->grab_djimotor[1]->daemon) ||
            !DaemonIsOnline(grab->actuator->grab_djimotor[2]->daemon))
        {
            return; // 还没上线，死等，不放行
        }

        // 记录三个 2006 当前坐标作为临时起跑线
        total_angle_init_R = grab->actuator->grab_djimotor[0]->measure.total_angle;
        total_angle_init_L = grab->actuator->grab_djimotor[1]->measure.total_angle;
        total_angle_init_M = grab->actuator->grab_djimotor[2]->measure.total_angle;

        last_r_angle = total_angle_init_R;
        last_l_angle = total_angle_init_L;
        cali_pitch = 0.0f;

        timeout_cnt = 0;
        cali_stage = CALI_STAGE_DM_WAIT_ZERO;
        first_run = 0;
    }

    // 3. 核心状态机
    switch (cali_stage)
    {
        case CALI_STAGE_DM_WAIT_ZERO:
        {
            timeout_cnt++;

            grab_ctrl_cmd->base_joint  = 0.0f;
            grab_ctrl_cmd->elbow_pitch = 0.0f;
            grab_ctrl_cmd->elbow_roll  = 0.0f;

            // 🚨 锁死手腕 3个电机 防止干涉！
            grab->actuator->R_target = total_angle_init_R;
            grab->actuator->L_target = total_angle_init_L;
            grab->actuator->M_target = total_angle_init_M;

            // 读取大臂真实姿态
            float curr_base   = grab->arm->grab_dmmotor[0]->measure.total_angle * RAD_2_DEGREE;
            float curr_elbow_r= grab->arm->grab_dmmotor[1]->measure.total_angle * RAD_2_DEGREE;
            float curr_elbow_p= grab->arm->grab_dmmotor[2]->measure.total_angle * RAD_2_DEGREE;

            // 到位检测
            if (fabsf(curr_base) < DM_HOMING_TOLERANCE &&
                fabsf(curr_elbow_r) < DM_HOMING_TOLERANCE &&
                fabsf(curr_elbow_p) < DM_HOMING_TOLERANCE)
            {
                LOGINFO("[GRAB] DM Motors reached 0. Starting DJI Wrist Cali.");
                timeout_cnt = 0;
                cali_stage = CALI_STAGE_WRIST_STALL;
            }
            // 超时判定
            else if (timeout_cnt > DM_CALI_MAX_TICKS)
            {
                LOGERROR("[GRAB] TIMEOUT! DM Arm failed to reach 0.");
                cali_stage = CALI_STAGE_ERROR;
            }
            break;
        }

        case CALI_STAGE_WRIST_STALL:
        {
            timeout_cnt++;

            // DM 大臂定在 0 度安全姿态
            grab_ctrl_cmd->base_joint  = 0.0f;
            grab_ctrl_cmd->elbow_pitch = 0.0f;
            grab_ctrl_cmd->elbow_roll  = 0.0f;

            // Roll 轴保持不动
            grab->actuator->M_target = total_angle_init_M;

            // Pitch 角度不断增加
            cali_pitch += WRIST_CALI_SPEED;

            // 🚨 同向结构计算目标位置
            grab->actuator->R_target = total_angle_init_R + (cali_pitch) * MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;
            grab->actuator->L_target = total_angle_init_L + (cali_pitch) * MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;

            // 堵转特征累积检测逻辑
            block_cnt++;
            if (block_cnt >= WRIST_CALI_CHECK_TICKS)
            {
                float curr_r = grab->actuator->grab_djimotor[0]->measure.total_angle;
                float curr_l = grab->actuator->grab_djimotor[1]->measure.total_angle;

                float diff_r = fabsf(curr_r - last_r_angle);
                float diff_l = fabsf(curr_l - last_l_angle);

                float curr_amp_r = fabsf((float)grab->actuator->grab_djimotor[0]->measure.real_current);
                float curr_amp_l = fabsf((float)grab->actuator->grab_djimotor[1]->measure.real_current);

                // 检测到双边 Pitch 电机微小位移 + 大电流
                if (diff_r < WRIST_CALI_TOLERANCE && diff_l < WRIST_CALI_TOLERANCE &&
                    curr_amp_r > WRIST_CALI_STALL_CURRENT && curr_amp_l > WRIST_CALI_STALL_CURRENT)
                {
                    // 💥 撞死限位！逆向解算真实的初始零点
                    float ratio_multiplier = MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;

                    total_angle_init_R = curr_r - (90.0f * ratio_multiplier);
                    total_angle_init_L = curr_l - (90.0f * ratio_multiplier);

                    grab->actuator->wrist_cali.state = CALI_SUCCESS;

                    // 对齐防抽搐
                    grab_ctrl_cmd->base_joint  = 0.0f;
                    grab_ctrl_cmd->elbow_pitch = 0.0f;
                    grab_ctrl_cmd->elbow_roll  = 0.0f;
                    grab_ctrl_cmd->wrist_pitch = 90.0f;
                    grab_ctrl_cmd->wrist_roll  = 0.0f;

                    LOGINFO("[GRAB] Pitch Cali SUCCESS! Limit is 90 deg.");
                    cali_stage = CALI_STAGE_DONE;
                }

                last_r_angle = curr_r;
                last_l_angle = curr_l;
                block_cnt = 0;
            }

            if (timeout_cnt > WRIST_CALI_MAX_TICKS && cali_stage != CALI_STAGE_DONE)
            {
                LOGERROR("[GRAB] TIMEOUT! Wrist stall not detected.");
                cali_stage = CALI_STAGE_ERROR;
            }
            break;
        }

        case CALI_STAGE_DONE:
        {
            return;
        }

        case CALI_STAGE_ERROR:
        {
            grab_ctrl_cmd->base_joint  = 0.0f;
            grab_ctrl_cmd->elbow_pitch = 0.0f;
            grab_ctrl_cmd->elbow_roll  = 0.0f;

            // 全员退回初始点防烧毁
            grab->actuator->R_target = total_angle_init_R;
            grab->actuator->L_target = total_angle_init_L;
            grab->actuator->M_target = total_angle_init_M;

            return;
        }

        default:
            cali_stage = CALI_STAGE_ERROR;
            break;
    }
}