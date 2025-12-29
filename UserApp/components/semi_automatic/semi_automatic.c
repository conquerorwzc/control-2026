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
 * @brief 启动半自动操作
 */
void StartSemiAutoOperation(void)
{
    if (semi_auto_instance != NULL) {
        semi_auto_instance->ctrl_cmd.is_running = 1;
        semi_auto_instance->ctrl_cmd.state = SEMI_AUTO_INSERT_MINERAL;  // 直接从第二步开始
        semi_auto_instance->ctrl_cmd.step_start_time = xTaskGetTickCount();
    }
}

/**
 * @brief 执行抬升龙门架操作（独立功能）
 */
void LiftGantryToTarget(void)
{
    if (semi_auto_instance != NULL && semi_auto_instance->gantry != NULL) {
        semi_auto_instance->gantry->Gantry_ctrl_cmd.z = semi_auto_instance->param.gantry_lift_pos;
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
        case SEMI_AUTO_INSERT_MINERAL:
            // 第一步：将矿物插入科技核心（移动底盘）
            if (instance->chassis != NULL) {
                // 设置底盘向前移动，执行插入操作
                instance->chassis->chassis_ctrl_cmd.vx = instance->param.chassis_forward_speed; // 向前移动
                // 假设插入操作需要2000ms完成
                if (elapsed_time >= 2000) {
                    instance->chassis->chassis_ctrl_cmd.vx = 0; // 停止移动
                    instance->ctrl_cmd.state = SEMI_AUTO_RAISE_ARM;
                    instance->ctrl_cmd.step_start_time = current_time;
                }
            } else {
                instance->ctrl_cmd.state = SEMI_AUTO_ERROR;
            }
            break;

        case SEMI_AUTO_RAISE_ARM:
            // 第二步：机械臂整体上抬
            if (instance->grab != NULL) {
                // 设置机械臂抬升角度（这里假设是肘部关节角度）
                instance->grab->grab_ctrl_cmd.elbow_pitch += 10.0f; // 增加10度
                // 假设上抬操作需要1000ms完成
                if (elapsed_time >= 1000) {
                    instance->ctrl_cmd.state = SEMI_AUTO_FLIP_HANDLE;
                    instance->ctrl_cmd.step_start_time = current_time;
                }
            } else {
                instance->ctrl_cmd.state = SEMI_AUTO_ERROR;
            }
            break;

        case SEMI_AUTO_FLIP_HANDLE:
            // 第三步：掰科技核心把手
            if (instance->grab != NULL) {
                // 设置掰把手的角度（这里假设是腕部关节角度）
                instance->grab->grab_ctrl_cmd.wrist_pitch = instance->param.handle_flip_angle;
                // 假设掰把手操作需要1200ms完成
                if (elapsed_time >= 1200) {
                    instance->ctrl_cmd.state = SEMI_AUTO_ROTATE_RIGHT;
                    instance->ctrl_cmd.step_start_time = current_time;
                }
            } else {
                instance->ctrl_cmd.state = SEMI_AUTO_ERROR;
            }
            break;

        case SEMI_AUTO_ROTATE_RIGHT:
            // 第四步：向右旋转5度
            if (instance->grab != NULL) {
                // 设置旋转角度（这里假设是基座关节角度）
                instance->grab->grab_ctrl_cmd.base_joint = instance->param.rotate_angle;
                // 假设旋转操作需要800ms完成
                if (elapsed_time >= 800) {
                    instance->ctrl_cmd.state = SEMI_AUTO_COMPLETE;
                    instance->ctrl_cmd.step_start_time = current_time;
                }
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