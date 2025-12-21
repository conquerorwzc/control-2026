//
// Created by ROG on 2025/11/17.
//
/* Private includes ----------------------------------------------------------*/
#include "grab.h"
#include "user_lib.h"
/* Private macro -------------------------------------------------------------*/
#define PULLEY_GEAR_RATIO 2.0f                  // 带轮传动比
#define BEVEL_GEAR_RATIO 1.6667f                // 锥齿轮传动比 5:3
#define MOTOR2006_REDUCTION_RATIO 36.0f         // 2006 ecd减速比36
/* Private define ------------------------------------------------------------*/
static GrabInstance *grab;
static Grab_Ctrl_Cmd_s *grab_ctrl_cmd;
static float total_angle_init_L = 0;
static float total_angle_init_R = 0;
static float total_angle_init_vedio_forward = 0;
static float total_angle_init_vedio_pitch = 0;
float a[5] = {0, 0, 0, 0, 0}; // 测试用电机目标位置
/* Private function prototypes -----------------------------------------------*/
GrabInstance *GrabInit(Grab_Init_Config_s *Grab_init_config); // 机械臂初始化，返回一个机械臂示例指针
void GrabTask(); // 机械臂任务函数
static void MotorTask(); // 电机任务函数
static void Grab_Position_Calculate(GrabInstance *grab); // 计算电机目标位置

/* Private user code ---------------------------------------------------------*/
/**
 * @brief 初始化机械臂
 */
GrabInstance *GrabInit(Grab_Init_Config_s *Grab_init_config) {
    GrabInstance *grab_instance = (GrabInstance *) zmalloc(sizeof(GrabInstance));
    grab_instance->actuator = (ActuatorInstance *) zmalloc(sizeof(ActuatorInstance));
    grab_instance->arm = (ArmInstance *) zmalloc(sizeof(ArmInstance));
    grab_instance->vedio = (VedioInstance *) zmalloc(sizeof(VedioInstance));

    grab_instance->actuator->grab_dmmotor[0] = DMMotorInit(&Grab_init_config->Grab_motor_config[5]); // v2
    grab_instance->arm->grab_dmmotor[0] = DMMotorInit(&Grab_init_config->Grab_motor_config[0]); // v3
    grab_instance->arm->grab_dmmotor[1] = DMMotorInit(&Grab_init_config->Grab_motor_config[1]); // v4
    grab_instance->arm->grab_dmmotor[2] = DMMotorInit(&Grab_init_config->Grab_motor_config[2]); // v4

    grab_instance->actuator->grab_djimotor[0] = DJIMotorInit(&Grab_init_config->Grab_motor_config[3]);
    grab_instance->actuator->grab_djimotor[1] = DJIMotorInit(&Grab_init_config->Grab_motor_config[4]);

    // grab_instance->vedio->grab_djimotor[0] = DJIMotorInit(&Grab_init_config->Grab_motor_config[6]);
    // grab_instance->vedio->grab_djimotor[1] = DJIMotorInit(&Grab_init_config->Grab_motor_config[7]);

    // 先赋值grab指针，再访问grab_instance中的成员
    grab = grab_instance;
    grab_ctrl_cmd = &grab->grab_ctrl_cmd;

    // 初始化电机初始角度，必须要发生在grab的赋值之后
    // osDelay(10);
    total_angle_init_L = grab->actuator->grab_djimotor[1]->measure.total_angle;
    total_angle_init_R = grab->actuator->grab_djimotor[0]->measure.total_angle;
    // total_angle_init_vedio_forward = grab->vedio->grab_djimotor[0]->measure.total_angle;
    // total_angle_init_vedio_pitch = grab->vedio->grab_djimotor[1]->measure.total_angle;
    return grab_instance;
}

/**
 * @brief 机械臂任务函数
 */
void GrabTask() {
    grab_ctrl_cmd = &grab->grab_ctrl_cmd;
    Grab_Position_Calculate(grab);
    MotorTask();
}

static void MotorTask() {
    if (grab_ctrl_cmd->grab_mode == GRAB_POWER_OFF) {
        DMMotorStop(grab->arm->grab_dmmotor[0]);
        DMMotorStop(grab->arm->grab_dmmotor[1]);
        DMMotorStop(grab->arm->grab_dmmotor[2]);

        DJIMotorStop(grab->actuator->grab_djimotor[0]);
        DJIMotorStop(grab->actuator->grab_djimotor[1]);
        DMMotorStop(grab->actuator->grab_dmmotor[0]);

        // DJIMotorStop(grab->vedio->grab_djimotor[0]);
        // DJIMotorStop(grab->vedio->grab_djimotor[1]);
    } else {
        DMMotorEnable(grab->arm->grab_dmmotor[0]);
        DMMotorEnable(grab->arm->grab_dmmotor[1]);
        DMMotorEnable(grab->arm->grab_dmmotor[2]);
        DJIMotorEnable(grab->actuator->grab_djimotor[0]);
        DJIMotorEnable(grab->actuator->grab_djimotor[1]);
        DMMotorEnable(grab->actuator->grab_dmmotor[0]);
        // DJIMotorEnable(grab->vedio->grab_djimotor[0]);
        // DJIMotorEnable(grab->vedio->grab_djimotor[1]);

        DMMotorSetPIDRef(grab->arm->grab_dmmotor[0], grab->arm->base_joint * DEGREE_2_RAD ); //弧度制，需要把度转弧度
        DMMotorSetPIDRef(grab->arm->grab_dmmotor[1], grab->arm->elbow_roll * DEGREE_2_RAD);
        DMMotorSetPIDRef(grab->arm->grab_dmmotor[2], grab->arm->elbow_pitch * DEGREE_2_RAD);

        DJIMotorSetPIDRef(grab->actuator->grab_djimotor[0], grab->actuator->R_target);
        DJIMotorSetPIDRef(grab->actuator->grab_djimotor[1], grab->actuator->L_target);


        DMMotorSetRef(grab->actuator->grab_dmmotor[0], grab->actuator->T);

        // DJIMotorSetPIDRef(grab->vedio->grab_djimotor[0], grab->vedio->F_target);
        // DJIMotorSetPIDRef(grab->vedio->grab_djimotor[1], grab->vedio->P_target);
    }
}

/**
 * @brief 根据pitch和roll角度解算出电机的转动角度
 * @param arm 机械臂结构体指针
 */
static void Grab_Position_Calculate(GrabInstance *grab) {
    /**
     * L = -pitch + roll
     * R = +pitch + roll
     */

    grab->actuator->R_target = total_angle_init_R + (
                                   grab->actuator->wrist_pitch + grab->actuator->wrist_roll * BEVEL_GEAR_RATIO) *
                               MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;
    grab->actuator->L_target = total_angle_init_L + (
                                   -grab->actuator->wrist_pitch + grab->actuator->wrist_roll * BEVEL_GEAR_RATIO) *
                               MOTOR2006_REDUCTION_RATIO * PULLEY_GEAR_RATIO;
    grab->vedio->F_target = total_angle_init_vedio_forward + grab->vedio->Vedio_forward * MOTOR2006_REDUCTION_RATIO;
    grab->vedio->P_target = total_angle_init_vedio_pitch + grab->vedio->Vedio_pitch;

    // grab->actuator->gripper_joint ;
}
