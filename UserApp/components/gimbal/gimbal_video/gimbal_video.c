/**
 * @file    gimbal_video.c
 * @brief   图传云台控制逻辑
 * Yaw: GM6020 (绝对值，无需标定)
 * Pitch: M3508 (相对值，双向堵转标定机械限位，带完整实车保护机制)
 */

#include "gimbal_video.h"
#include "user_lib.h"
#include <math.h>

// 全局实例指针
static VideoGimbalInstance *video_gimbal;

/* ==================== 极度舒适的实车标定参数 ==================== */
#define VIDEO_CALI_SPEED_P          0.5f    // M3508 俯仰寻零速度 (度/tick)
#define VIDEO_CALI_STALL_CURRENT_P  2500.0f // M3508 堵转认定电流 (原始值)
#define VIDEO_CALI_STALL_SPEED      20.0f   // 转速阈值 (rpm)
#define VIDEO_CALI_STALL_TICKS      300     // 连续 300 帧判定成功 (0.3秒)
#define VIDEO_CALI_MAX_TICKS        5000    // 单次寻零超时时间 (5秒)
#define VIDEO_CALI_SLIP_LIMIT_P     60.0f   // P电机目标角度最大超前量
#define VIDEO_CALI_MAXOUT_P         6000.0f // 寻零期间，P电机最大输出 (温柔摸墙)
#define VIDEO_SAFE_RANGE_RATIO      0.95f   // 软限位安全系数
// Yaw轴：系数 0.05f 对应按键按下时 50度/秒 (1000Hz * 1.0 * 0.05)
#define VIDEO_YAW_SPEED_SENS    0.05f
// Pitch轴：系数 0.0005f 对应按键按下时，从最底到最顶需要 2 秒 (1000Hz * 1.0 * 0.0005 = 0.5/s)
#define VIDEO_PITCH_SPEED_SENS  0.0005f
/* ==================== 初始化 ==================== */
VideoGimbalInstance *VideoGimbalInit(VideoGimbal_Init_Config_s *config)
{
    VideoGimbalInstance *inst = (VideoGimbalInstance *)zmalloc(sizeof(VideoGimbalInstance));

    inst->yaw_motor = DJIMotorInit(&config->yaw_motor_config);
    inst->pitch_motor = DJIMotorInit(&config->pitch_motor_config);

    inst->yaw_zero_angle = config->yaw_zero_angle;
    inst->yaw_zero_cali_done = 1;

    inst->video_cali_state = VIDEO_CALI_START;
    inst->P_target = 0.0f;

    video_gimbal = inst;
    return inst;
}

/* ==================== 主任务 ==================== */
void VideoGimbalTask(void)
{
    if (video_gimbal == NULL || video_gimbal->yaw_motor == NULL || video_gimbal->pitch_motor == NULL) return;

    // 断电保护逻辑
    if (video_gimbal->ctrl_cmd.power == VIDEO_POWER_OFF) {
        DJIMotorStop(video_gimbal->yaw_motor);
        DJIMotorStop(video_gimbal->pitch_motor);
        video_gimbal->video_cali_state = VIDEO_CALI_START; // 重置标定状态
        return;
    } else {
        DJIMotorEnable(video_gimbal->yaw_motor);
        DJIMotorEnable(video_gimbal->pitch_motor);
    }

    // --------------------------------------------------------
    // 【 Yaw 轴控制逻辑 】- GM6020
    // --------------------------------------------------------
    video_gimbal->Video_yaw += video_gimbal->ctrl_cmd.video_yaw * VIDEO_YAW_SPEED_SENS;
    video_gimbal->Y_target = video_gimbal->yaw_zero_angle + video_gimbal->Video_yaw;
    DJIMotorSetPIDRef(video_gimbal->yaw_motor, video_gimbal->Y_target);

    // --------------------------------------------------------
    // 【 Pitch 轴状态机 】- M3508
    // --------------------------------------------------------
    static uint16_t stall_counter = 0;
    static uint16_t timeout_counter = 0;

    // 用于保存初始的最大输出限制，标定完成后恢复
    static float orig_speed_max_out = 0.0f;
    static float orig_current_max_out = 0.0f;

    float pitch_speed   = video_gimbal->pitch_motor->measure.speed_aps;
    float pitch_angle   = video_gimbal->pitch_motor->measure.total_angle;
    float pitch_current = video_gimbal->pitch_motor->measure.real_current;

    switch (video_gimbal->video_cali_state)
    {
        case VIDEO_CALI_START:
            stall_counter = 0;
            timeout_counter = 0;
            video_gimbal->video_cali_state = VIDEO_CALI_START_PITCH;
            break;

        case VIDEO_CALI_WAIT_BTN:
            if (video_gimbal->ctrl_cmd.video_cali == 1) {
                video_gimbal->video_cali_state = VIDEO_CALI_START_PITCH;
            }
            break;

        case VIDEO_CALI_START_PITCH:
            video_gimbal->P_target = pitch_angle;
            stall_counter = 0;
            timeout_counter = 0;

            // 【黑魔法：保存初始值，并限制输出】
            // 注意：如果你的 controller.h 里成员变量大写了 (例如 MaxOut)，请自行将 max_out 改为 MaxOut
            orig_speed_max_out = video_gimbal->pitch_motor->motor_controller.speed_PID.MaxOut;
            orig_current_max_out = video_gimbal->pitch_motor->motor_controller.current_PID.MaxOut;

            video_gimbal->pitch_motor->motor_controller.speed_PID.MaxOut= VIDEO_CALI_MAXOUT_P;
            video_gimbal->pitch_motor->motor_controller.current_PID.MaxOut = VIDEO_CALI_MAXOUT_P;

            video_gimbal->video_cali_state = VIDEO_CALI_FIND_MAX;
            break;

        case VIDEO_CALI_FIND_MAX:
            timeout_counter++;
            if (timeout_counter > VIDEO_CALI_MAX_TICKS) {
                video_gimbal->video_cali_state = VIDEO_CALI_ERROR;
                break;
            }

            // 1. 目标角度累加 (斜坡)
            video_gimbal->P_target += VIDEO_CALI_SPEED_P;

            // 2. 超前量限制 (防止 PID 误差过大)
            if (video_gimbal->P_target > pitch_angle + VIDEO_CALI_SLIP_LIMIT_P) {
                video_gimbal->P_target = pitch_angle + VIDEO_CALI_SLIP_LIMIT_P;
            }

            DJIMotorSetPIDRef(video_gimbal->pitch_motor, video_gimbal->P_target);

            // 3. 电流速度双判定
            if (fabs(pitch_speed) < VIDEO_CALI_STALL_SPEED && fabs(pitch_current) > VIDEO_CALI_STALL_CURRENT_P) {
                stall_counter++;
            } else {
                stall_counter = 0;
            }

            if (stall_counter > VIDEO_CALI_STALL_TICKS) {
                video_gimbal->video_max_p = pitch_angle;
                stall_counter = 0;
                timeout_counter = 0; // 重置超时，准备找下限位
                video_gimbal->video_cali_state = VIDEO_CALI_FIND_MIN;
            }
            break;

        case VIDEO_CALI_FIND_MIN:
            timeout_counter++;
            if (timeout_counter > VIDEO_CALI_MAX_TICKS) {
                video_gimbal->video_cali_state = VIDEO_CALI_ERROR;
                break;
            }

            video_gimbal->P_target -= VIDEO_CALI_SPEED_P;

            // 超前量限制 (向下寻找，限制负向误差)
            if (video_gimbal->P_target < pitch_angle - VIDEO_CALI_SLIP_LIMIT_P) {
                video_gimbal->P_target = pitch_angle - VIDEO_CALI_SLIP_LIMIT_P;
            }

            DJIMotorSetPIDRef(video_gimbal->pitch_motor, video_gimbal->P_target);

            if (fabs(pitch_speed) < VIDEO_CALI_STALL_SPEED && fabs(pitch_current) > VIDEO_CALI_STALL_CURRENT_P) {
                stall_counter++;
            } else {
                stall_counter = 0;
            }

            if (stall_counter > VIDEO_CALI_STALL_TICKS) {
                video_gimbal->video_min_p = pitch_angle;
                stall_counter = 0;

                // 【用完即焚：恢复正常的 PID 输出上限】
                video_gimbal->pitch_motor->motor_controller.speed_PID.MaxOut = orig_speed_max_out;
                video_gimbal->pitch_motor->motor_controller.current_PID.MaxOut = orig_current_max_out;

                // 应用安全系数缩进物理限位 (避免正常运动时撞墙)
                float range = video_gimbal->video_max_p - video_gimbal->video_min_p;
                float safe_margin = range * (1.0f - VIDEO_SAFE_RANGE_RATIO) / 2.0f;
                video_gimbal->video_max_p -= safe_margin;
                video_gimbal->video_min_p += safe_margin;

                // 回正
                video_gimbal->P_target = (video_gimbal->video_max_p + video_gimbal->video_min_p) / 2.0f;
                video_gimbal->Video_pitch = 0.5f;
                video_gimbal->video_cali_state = VIDEO_CALI_DONE;
            }
            break;

    case VIDEO_CALI_DONE:
        // 1. 把传入的指令作为“增量”加到基础比例上
        // 摇杆往上推为正，往下推为负，松手为 0
        video_gimbal->Video_pitch += video_gimbal->ctrl_cmd.video_pitch * VIDEO_PITCH_SPEED_SENS;
        // 2. 限制比例池永远在 0.0 ~ 1.0 之间 (0% 到 100%)
        if (video_gimbal->Video_pitch > 1.0f) video_gimbal->Video_pitch = 1.0f;
        if (video_gimbal->Video_pitch < 0.0f) video_gimbal->Video_pitch = 0.0f;

        // 3. 映射出实际的物理目标角度
        float pitch_range = video_gimbal->video_max_p - video_gimbal->video_min_p;
        video_gimbal->P_target = video_gimbal->video_min_p + (pitch_range * video_gimbal->Video_pitch);

        // 4. 下发给底层
        DJIMotorSetPIDRef(video_gimbal->pitch_motor, video_gimbal->P_target);
        break;

        case VIDEO_CALI_ERROR:
            // 标定超时或异常，切断输出
            DJIMotorStop(video_gimbal->pitch_motor);
            break;

        default:
            break;
    }
}