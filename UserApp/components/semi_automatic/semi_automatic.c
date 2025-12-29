#include "semi_automatic.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "stdlib.h"
#include "string.h"
#include "chassis.h"

// 半自动操作实例
static SemiAutoInstance* semi_auto_instance = NULL;

// 半自动操作任务函数
static void SemiAuto_Run(SemiAutoInstance* instance);

/**
 * @brief 半自动操作初始化
 * @param init_config 初始化配置
 * @return SemiAutoInstance* 半自动操作实例指针
 */
SemiAutoInstance* SemiAutoInit(SemiAuto_Init_Config_s* init_config)
{
    if (semi_auto_instance != NULL) {
        return semi_auto_instance; // 已经初始化过
    }

    semi_auto_instance = (SemiAutoInstance*)malloc(sizeof(SemiAutoInstance));
    if (semi_auto_instance == NULL) {
        return NULL;
    }
    memset(semi_auto_instance, 0, sizeof(SemiAutoInstance));

    // 保存参数配置
    semi_auto_instance->param = init_config->param;
    
    // 初始化控制命令
    semi_auto_instance->ctrl_cmd.state = SEMI_AUTO_IDLE;
    semi_auto_instance->ctrl_cmd.is_running = 0;
    semi_auto_instance->ctrl_cmd.manual_stop = 0;

    return semi_auto_instance;
}

/**
 * @brief 半自动操作任务
 */
void SemiAutoTask(void)
{
    if (semi_auto_instance != NULL) {
        SemiAuto_Run(semi_auto_instance);
    }
}

/**
 * @brief 启动半自动操作 - 抬升龙门架
 */
void StartSemiAutoOperation(void)
{
    if (semi_auto_instance != NULL) {
        semi_auto_instance->ctrl_cmd.is_running = 1;
        semi_auto_instance->ctrl_cmd.state = SEMI_AUTO_RAISE_GANTRY;  // 从抬升龙门架开始
        semi_auto_instance->ctrl_cmd.step_start_time = xTaskGetTickCount();
    }
}

/**
 * @brief 停止半自动操作
 */
void StopSemiAutoOperation(void)
{
    if (semi_auto_instance != NULL) {
        semi_auto_instance->ctrl_cmd.is_running = 0;
        semi_auto_instance->ctrl_cmd.state = SEMI_AUTO_IDLE;
    }
}

/**
 * @brief 重置半自动操作
 */
void ResetSemiAutoOperation(void)
{
    if (semi_auto_instance != NULL) {
        semi_auto_instance->ctrl_cmd.is_running = 0;
        semi_auto_instance->ctrl_cmd.state = SEMI_AUTO_IDLE;
        semi_auto_instance->ctrl_cmd.manual_stop = 0;
    }
}

/**
 * @brief 启动龙门架抬升
 */
void StartGantryLift(void)
{
    if (semi_auto_instance != NULL) {
        semi_auto_instance->ctrl_cmd.is_running = 1;
        semi_auto_instance->ctrl_cmd.state = SEMI_AUTO_RAISE_GANTRY;
        semi_auto_instance->ctrl_cmd.step_start_time = xTaskGetTickCount();
    }
}

/**
 * @brief 启动底盘移动
 */
void StartChassisMove(void)
{
    if (semi_auto_instance != NULL) {
        semi_auto_instance->ctrl_cmd.is_running = 1;
        semi_auto_instance->ctrl_cmd.state = SEMI_AUTO_MOVE_CHASSIS;
        semi_auto_instance->ctrl_cmd.step_start_time = xTaskGetTickCount();
    }
}

/**
 * @brief 启动机械臂上抬
 */
void StartArmRaise(void)
{
    if (semi_auto_instance != NULL) {
        semi_auto_instance->ctrl_cmd.is_running = 1;
        semi_auto_instance->ctrl_cmd.state = SEMI_AUTO_ARM_RAISE;
        semi_auto_instance->ctrl_cmd.step_start_time = xTaskGetTickCount();
    }
}

/**
 * @brief 启动掰把手
 */
void StartArmFlip(void)
{
    if (semi_auto_instance != NULL) {
        semi_auto_instance->ctrl_cmd.is_running = 1;
        semi_auto_instance->ctrl_cmd.state = SEMI_AUTO_ARM_FLIP;
        semi_auto_instance->ctrl_cmd.step_start_time = xTaskGetTickCount();
    }
}

/**
 * @brief 启动旋转
 */
void StartArmRotate(void)
{
    if (semi_auto_instance != NULL) {
        semi_auto_instance->ctrl_cmd.is_running = 1;
        semi_auto_instance->ctrl_cmd.state = SEMI_AUTO_ARM_ROTATE;
        semi_auto_instance->ctrl_cmd.step_start_time = xTaskGetTickCount();
    }
}

/**
 * @brief 半自动操作运行函数
 * @param instance 半自动操作实例
 */
static void SemiAuto_Run(SemiAutoInstance* instance)
{
    if (instance == NULL || !instance->ctrl_cmd.is_running) {
        return;
    }

    uint32_t current_time = xTaskGetTickCount();
    uint32_t elapsed_time = current_time - instance->ctrl_cmd.step_start_time;

    // 根据当前状态执行相应的操作
    switch (instance->ctrl_cmd.state)
    {
        case SEMI_AUTO_RAISE_GANTRY:
            // 第一步：抬升龙门架
            if (instance->gantry != NULL) {
                // 设置龙门架抬升位置
                instance->gantry->Gantry_ctrl_cmd.z = instance->param.gantry_lift_pos;
                // 立即跳转到空闲状态，等待手动微调
                instance->ctrl_cmd.state = SEMI_AUTO_IDLE;
                instance->ctrl_cmd.is_running = 0;
            } else {
                instance->ctrl_cmd.state = SEMI_AUTO_ERROR;
            }
            break;

        case SEMI_AUTO_MOVE_CHASSIS:
            // 第二步：移动底盘
            if (instance->chassis != NULL) {
                // 设置底盘向前移动
                instance->chassis->chassis_ctrl_cmd.vx = instance->param.chassis_forward_speed;
                // 立即跳转到空闲状态，等待手动微调
                instance->chassis->chassis_ctrl_cmd.vx = 0; // 立即停止
                instance->ctrl_cmd.state = SEMI_AUTO_IDLE;
                instance->ctrl_cmd.is_running = 0;
            } else {
                instance->ctrl_cmd.state = SEMI_AUTO_ERROR;
            }
            break;

        case SEMI_AUTO_ARM_RAISE:
            // 第三步：机械臂上抬
            if (instance->grab != NULL) {
                // 机械臂上抬 - 设置目标角度
                instance->grab->grab_ctrl_cmd.elbow_pitch = instance->param.elbow_pitch_angle;
                
                // 立即跳转到空闲状态，等待手动微调
                instance->ctrl_cmd.state = SEMI_AUTO_IDLE;
                instance->ctrl_cmd.is_running = 0;
            } else {
                instance->ctrl_cmd.state = SEMI_AUTO_ERROR;
            }
            break;
            
        case SEMI_AUTO_ARM_FLIP:
            // 第四步：掰把手
            if (instance->grab != NULL) {
                // 把手掰动 - 设置目标角度
                instance->grab->grab_ctrl_cmd.wrist_pitch = instance->param.wrist_pitch_angle;
                
                // 立即跳转到空闲状态，等待手动微调
                instance->ctrl_cmd.state = SEMI_AUTO_IDLE;
                instance->ctrl_cmd.is_running = 0;
            } else {
                instance->ctrl_cmd.state = SEMI_AUTO_ERROR;
            }
            break;
            
        case SEMI_AUTO_ARM_ROTATE:
            // 第五步：旋转
            if (instance->grab != NULL) {
                // 旋转 - 设置目标角度
                instance->grab->grab_ctrl_cmd.base_joint = instance->param.base_joint_angle;
                
                // 立即跳转到完成状态
                instance->ctrl_cmd.state = SEMI_AUTO_COMPLETE;
                instance->ctrl_cmd.step_start_time = current_time;
            } else {
                instance->ctrl_cmd.state = SEMI_AUTO_ERROR;
            }
            break;

        case SEMI_AUTO_COMPLETE:
            // 操作完成，保持状态一段时间后返回空闲
            if (elapsed_time >= 2000) { // 保持完成状态2秒后回到空闲
                instance->ctrl_cmd.state = SEMI_AUTO_IDLE;
                instance->ctrl_cmd.is_running = 0;
            }
            break;

        case SEMI_AUTO_ERROR:
            // 错误状态，停止操作
            instance->ctrl_cmd.is_running = 0;
            break;

        default:
            // 默认情况回到空闲状态
            instance->ctrl_cmd.state = SEMI_AUTO_IDLE;
            instance->ctrl_cmd.is_running = 0;
            break;
    }
}