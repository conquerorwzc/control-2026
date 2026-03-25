/**
 * @file    gimbal_video.c
 * @brief   图传云台独立组件
 *          Yaw : GM6020（绝对值编码器，上电后直接读角度，支持固定物理零点标定）
 *          Pitch: M3508 （相对编码器，上电后双向堵转自动标定）
 */

#include "gimbal_video.h"
#include "user_lib.h"
#include <math.h>

/* ==================== 宏参数 ==================== */
#define VIDEO_CALI_SPEED_P       0.5f      // Pitch 标定每 tick 移动角度
#define VIDEO_CALI_STALL_CURRENT_P 2500    // Pitch 堵转电流阈值
#define VIDEO_CALI_MAX_TICKS     5000      // 单次寻零超时
#define VIDEO_CALI_SLIP_LIMIT_P  60.0f     // P 电机目标超前量限制
#define VIDEO_CALI_MAXOUT_P      6000.0f   // 标定期间 P 电机最大输出
#define VIDEO_SAFE_RANGE_RATIO   0.95f     // Pitch 安全范围比例
#define VIDEO_CALI_STALL_SPEED   20.0f     // 转速低于此值才判堵转 (APS)
#define VIDEO_CALI_STALL_TICKS   300       // 连续 300 帧判定堵转成功
/* ==================== 图传灵敏度调节宏 ==================== */
// Yaw 轴转速基准：ctrl_cmd->video_yaw == 1 时（即单独按下 B/V 键）对应的物理转速
// 鼠标是连续输入，实际转速 = mouse.x × VIDEO_MOUSE_YAW_SENS × VIDEO_YAW_SPEED_DPS
#define VIDEO_YAW_SPEED_DPS      7.0f  // 单独按 B/V 键时的 Yaw 转速 (°/s)
#define VIDEO_YAW_LPF            0.01f   // Yaw 轴软着陆平滑系数 (0.01~1.0，越小越顺滑，越大越跟手)

// Pitch 轴：ctrl_cmd->video_pitch 每单位对应的行程百分比累加速度（%/帧）
#define VIDEO_PITCH_SENSITIVITY  0.001f
#define VIDEO_PITCH_LPF          0.15f   // Pitch 轴软着陆平滑系数
/* ==================== 模块私有变量 ==================== */
static VideoGimbalInstance *video_gimbal;
static VideoGimbal_Ctrl_Cmd_s *ctrl_cmd;

// Yaw: GM6020 绝对编码器记录的运行时零点
static float total_angle_init_yaw   = 0.0f;
// Pitch: M3508 相对编码器记录的两端限位中心
static float total_angle_init_pitch = 0.0f;
static float original_max_out = 0.0f;
/* ==================== 前向声明 ==================== */
static void VideoCalibrationTask(void);
static void VideoPositionCalculate(void);
static void VideoMotorTask(void);
static void VideoCaliCheck(void);

/* ==================== 初始化 ==================== */
VideoGimbalInstance *VideoGimbalInit(VideoGimbal_Init_Config_s *config)
{
    VideoGimbalInstance *inst =
        (VideoGimbalInstance *)zmalloc(sizeof(VideoGimbalInstance));

    inst->yaw_motor   = DJIMotorInit(&config->yaw_motor_config);
    inst->pitch_motor = DJIMotorInit(&config->pitch_motor_config);

    inst->video_cali_state = VIDEO_CALI_START;

    // 从配置中读取 Yaw 零点：非零则为固定物理零点，0.0f 则用上电位置
    inst->yaw_zero_angle     = config->yaw_zero_angle;
    inst->yaw_zero_cali_done = (config->yaw_zero_angle != 0.0f) ? 1 : 0;
    inst->Video_yaw   = 0.0f;
    inst->Video_pitch = 0.5f;

    video_gimbal = inst;
    ctrl_cmd     = &inst->ctrl_cmd;

    return inst;
}

/* ==================== 主任务（200 Hz 调度） ==================== */
void VideoGimbalTask(void)
{
    if (video_gimbal == NULL) return;

    VideoCaliCheck();         // 处理上层标定触发命令

    if (video_gimbal->video_cali_state != VIDEO_CALI_DONE)
    {
        VideoCalibrationTask();  // 标定期间：状态机控制目标
    }
    else
    {
        // Yaw：以°/s 速度累积，除以任务频率(200Hz)换算为每帧增量
        video_gimbal->Video_yaw += ctrl_cmd->video_yaw * VIDEO_YAW_SPEED_DPS / 200.0f;

        // Pitch：0~1 百分比限幅
        video_gimbal->Video_pitch += ctrl_cmd->video_pitch * VIDEO_PITCH_SENSITIVITY;
        if (video_gimbal->Video_pitch > 1.0f) video_gimbal->Video_pitch = 1.0f;
        if (video_gimbal->Video_pitch < 0.0f) video_gimbal->Video_pitch = 0.0f;

        VideoPositionCalculate(); // LPF 软着陆解算
    }

    VideoMotorTask(); // 统一下发电机
}

/* ==================== 标定命令检查 ==================== */
static void VideoCaliCheck(void)
{
    // Pitch 双向堵转标定触发
    if (ctrl_cmd->video_cali == 1)
    {
        if (video_gimbal->video_cali_state == VIDEO_CALI_WAIT_BTN ||
            video_gimbal->video_cali_state == VIDEO_CALI_DONE)
        {
            video_gimbal->video_cali_state = VIDEO_CALI_START_PITCH;
        }
        ctrl_cmd->video_cali = 0;
    }
}

/* ==================== 标定状态机 ==================== */
static void VideoCalibrationTask(void)
{
    if (video_gimbal->yaw_motor == NULL || video_gimbal->pitch_motor == NULL) return;

    // 1. 断电：状态机归位，控制量清零；但 Yaw 零点标定数据保留
    if (ctrl_cmd->power == VIDEO_POWER_OFF)
    {
        video_gimbal->video_cali_state = VIDEO_CALI_START;
        video_gimbal->Video_yaw = 0.0f;
        ctrl_cmd->video_yaw   = 0.0f;
        ctrl_cmd->video_pitch = 0.0f;
        return;
    }

    // 2. 掉线保护（M3508 Pitch 掉线也需要重置，GM6020 Yaw 重连后 total_angle 仍有效）
    if (!DaemonIsOnline(video_gimbal->yaw_motor->daemon) ||
        !DaemonIsOnline(video_gimbal->pitch_motor->daemon))
    {
        video_gimbal->video_cali_state = VIDEO_CALI_START;
        video_gimbal->Video_yaw = 0.0f;
        ctrl_cmd->video_yaw   = 0.0f;
        ctrl_cmd->video_pitch = 0.0f;
        return;
    }

    // 3. 已完成：直接返回
    if (video_gimbal->video_cali_state == VIDEO_CALI_DONE) return;

    static float cali_target_p = 0.0f;
    static uint16_t p_stall_cnt = 0;
    static uint32_t timeout_cnt = 0;
    static uint8_t p_done = 0;
    static float hold_target_p = 0.0f;

    float curr_y   = video_gimbal->yaw_motor->measure.total_angle;
    float curr_p   = video_gimbal->pitch_motor->measure.total_angle;
    float curr_amp = fabsf((float)video_gimbal->pitch_motor->measure.real_current);
    float curr_spd = fabsf((float)video_gimbal->pitch_motor->measure.speed_aps);

    switch (video_gimbal->video_cali_state)
    {
    case VIDEO_CALI_START:
        // GM6020 Yaw 零点：已标定用固定物理角度，未标定用当前位置
        if (video_gimbal->yaw_zero_cali_done)
            total_angle_init_yaw = video_gimbal->yaw_zero_angle;
        else
            total_angle_init_yaw = curr_y;

        video_gimbal->Video_yaw = 0.0f;
        video_gimbal->Y_target  = curr_y; // 先锁当前，LPF 软着陆到零点
        hold_target_p           = curr_p;
        video_gimbal->P_target  = hold_target_p;

        video_gimbal->video_cali_state = VIDEO_CALI_WAIT_BTN;
        break;

    case VIDEO_CALI_WAIT_BTN:
        // Pitch 挂起在上电位置防下垂，Yaw 可以正常操控
        video_gimbal->P_target = hold_target_p;
        break;

    case VIDEO_CALI_START_PITCH:
        cali_target_p = curr_p;
        p_stall_cnt = 0; timeout_cnt = 0; p_done = 0;
        original_max_out = video_gimbal->pitch_motor->motor_controller.speed_PID.MaxOut;
        video_gimbal->video_cali_state = VIDEO_CALI_FIND_MAX;
        break;

    case VIDEO_CALI_FIND_MAX:
    case VIDEO_CALI_FIND_MIN:
    {
        timeout_cnt++;
        video_gimbal->pitch_motor->motor_controller.speed_PID.MaxOut = VIDEO_CALI_MAXOUT_P;

        float dir = (video_gimbal->video_cali_state == VIDEO_CALI_FIND_MAX) ? 1.0f : -1.0f;

        if (!p_done) cali_target_p += dir * VIDEO_CALI_SPEED_P;

        // 虚拟弹簧：防止目标太超前被卡死
        if (cali_target_p > curr_p + VIDEO_CALI_SLIP_LIMIT_P)
            cali_target_p = curr_p + VIDEO_CALI_SLIP_LIMIT_P;
        if (cali_target_p < curr_p - VIDEO_CALI_SLIP_LIMIT_P)
            cali_target_p = curr_p - VIDEO_CALI_SLIP_LIMIT_P;

        video_gimbal->P_target = cali_target_p;

        // 堵转判断
        if (!p_done)
        {
            if (curr_spd < VIDEO_CALI_STALL_SPEED && curr_amp > VIDEO_CALI_STALL_CURRENT_P)
                p_stall_cnt++;
            else
                p_stall_cnt = 0;

            if (p_stall_cnt > VIDEO_CALI_STALL_TICKS)
            {
                p_done = 1;
                if (video_gimbal->video_cali_state == VIDEO_CALI_FIND_MAX)
                    video_gimbal->video_max_p = curr_p;
                else
                    video_gimbal->video_min_p = curr_p;
            }
        }

        // 阶段切换
        if (p_done)
        {
            if (video_gimbal->video_cali_state == VIDEO_CALI_FIND_MAX)
            {
                video_gimbal->video_cali_state = VIDEO_CALI_FIND_MIN;
                p_done = 0; p_stall_cnt = 0; timeout_cnt = 0;
            }
            else
            {
                // 完成：计算 Pitch 绝对零点为量程中心
                total_angle_init_pitch =
                    (video_gimbal->video_max_p + video_gimbal->video_min_p) / 2.0f;

                video_gimbal->Video_pitch = 0.5f;
                ctrl_cmd->video_pitch = 0.0f;
                video_gimbal->pitch_motor->motor_controller.speed_PID.MaxOut = 15000.0f;
                video_gimbal->video_cali_state = VIDEO_CALI_DONE;
                video_gimbal->pitch_motor->motor_controller.speed_PID.MaxOut = original_max_out;
            }
        }

        if (timeout_cnt > VIDEO_CALI_MAX_TICKS)
            video_gimbal->video_cali_state = VIDEO_CALI_ERROR;
        break;
    }

    case VIDEO_CALI_ERROR:
        video_gimbal->Y_target = curr_y;
        video_gimbal->P_target = curr_p;
        break;

    default:
        break;
    }
}

/* ==================== 位置解算（LPF 软着陆） ==================== */
static void VideoPositionCalculate(void)
{
    VideoCaliState_e state = video_gimbal->video_cali_state;

    // Yaw：只要不是初始 START/ERROR 就可以解算
    // Video_yaw 已经是度(°)单位，直接叠加零点即可
    if (state != VIDEO_CALI_START && state != VIDEO_CALI_ERROR)
    {
        float target_y = total_angle_init_yaw + video_gimbal->Video_yaw;
        video_gimbal->Y_target += (target_y - video_gimbal->Y_target) * VIDEO_YAW_LPF;
    }

    // Pitch：仅标定完才解算
    if (state == VIDEO_CALI_DONE)
    {
        float stroke = video_gimbal->video_max_p - video_gimbal->video_min_p;
        float margin = (1.0f - VIDEO_SAFE_RANGE_RATIO) / 2.0f;
        float safe_p = margin + video_gimbal->Video_pitch * VIDEO_SAFE_RANGE_RATIO;

        if (safe_p < margin)           safe_p = margin;
        if (safe_p > 1.0f - margin)    safe_p = 1.0f - margin;

        float target_p = video_gimbal->video_min_p + stroke * safe_p;
        video_gimbal->P_target += (target_p - video_gimbal->P_target) * VIDEO_PITCH_LPF;
    }
}

/* ==================== 电机使能/停止下发 ==================== */
static void VideoMotorTask(void)
{
    if (ctrl_cmd->power == VIDEO_POWER_OFF)
    {
        // 如果之前已经标定完了，就保留 DONE 状态，不要重置！
        if (video_gimbal->video_cali_state != VIDEO_CALI_DONE) {
            video_gimbal->video_cali_state = VIDEO_CALI_START;
        }
        video_gimbal->Video_yaw = 0.0f;
        ctrl_cmd->video_yaw   = 0.0f;
        ctrl_cmd->video_pitch = 0.0f;
        if (video_gimbal->yaw_motor != NULL) DJIMotorStop(video_gimbal->yaw_motor);
        if (video_gimbal->pitch_motor != NULL) DJIMotorStop(video_gimbal->pitch_motor);
        return;
    }

    // Yaw（GM6020）：在线则使能
    if (video_gimbal->yaw_motor != NULL)
    {
        if (DaemonIsOnline(video_gimbal->yaw_motor->daemon))
        {
            DJIMotorEnable(video_gimbal->yaw_motor);
            DJIMotorSetPIDRef(video_gimbal->yaw_motor, video_gimbal->Y_target);
        }
        else
        {
            DJIMotorStop(video_gimbal->yaw_motor);
        }
    }

    // Pitch（M3508）：在线则使能
    if (video_gimbal->pitch_motor != NULL)
    {
        if (DaemonIsOnline(video_gimbal->pitch_motor->daemon))
        {
            DJIMotorEnable(video_gimbal->pitch_motor);
            DJIMotorSetPIDRef(video_gimbal->pitch_motor, video_gimbal->P_target);
        }
        else
        {
            DJIMotorStop(video_gimbal->pitch_motor);
        }
    }
}
