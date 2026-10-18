#include "robot.h"

#include "FreeRTOS.h"
#include "can_comm.h"
#include "general_def.h"
#include "master_process.h"
#include "new_RC_VT13.h"
#include "robot_config.h"
#include "task.h"
#include "user_lib.h"

#include <math.h>

static RobotInstance *robot;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;
static uint8_t nav_cmd_valid;
static Gimbal_Ctrl_Cmd_s *gimbal_ctrl_cmd;
static Shoot_Ctrl_Cmd_s *shoot_ctrl_cmd;
static Vision_Receive_s *vision_recv_data;
// static navigator_recv_t* navigator_data;
#ifdef USE_DUAL_RC
static Send_Data_RC *rc_data_old;
static RC_ctrl_t *rc_data;
static RC_ctrl_t *rc_data_last; // 遥控器数据,初始化时返回
#elifdef USE_DUAL_RC_NEW
static Send_Data_RC_NEW *rc_data_new;
static VT13_RC_t *vt13_rc_data;
#endif
static Sentry_Cmd_t sentry_cmd = {0};
static SuperCapMode supercap_mode = SAFETY_MODE;
float trigger_time = 0; // 触发时间
static float angle = 0;
CANCommInstance *can_comm_instance = NULL;
static Referee_Data *referee_data;
static float time = 0; // 判断按钮按下需要重复读取时间，这里简化成一次读取

static float vx_initial;       // x轴输入控制量
static float vy_initial;       // y轴输入控制量
// static  DJIMotorInstance* debug_motor;

static uint16_t EncodeBulletSpeedToU16(float speed_mps)
{
    if (speed_mps <= 0.0f)
        return 0u;
    if (speed_mps >= 30.0f)
        return (uint16_t)(30.0f * 100.0f);
    return (uint16_t)(speed_mps * 100.0f);
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
uint8_t has_non_zero_data(const Vision_Receive_s *data)
{
    // 空指针检查
    if (data == NULL)
    {
        return 0; // 或根据需求返回错误码
    }
    // 简化逻辑：只要任意字段非零，返回1；否则返回0
    return (data->gimbal_receive.pitch != 0) || (data->gimbal_receive.yaw != 0) || (data->shoot_receive.fire_flag != 0);
}
static void CalcOffsetAngle()
{
#ifdef USE_DUAL_RC
    angle = rc_data_old->Yaw_motor_angle;
#elifdef USE_DUAL_RC_NEW
    angle = rc_data_new->Yaw_motor_angle;
#endif
    float delta = angle - YAW_ALIGN_ANGLE;
    if (delta > 180.0f) delta -= 360.0f;
    else if (delta <= -180.0f)  delta += 360.0f;
    // 双板上传的是原始 yaw 编码器角；转换为云台相对底盘的逆时针正偏角。
    chassis_ctrl_cmd->offset_angle = -delta;
}

static void SentryRefereeSend()
{
    sentry_cmd.fields.confirm_respawn = 1;
    sentry_cmd.fields.confirm_instant_respawn = 0;
    sentry_cmd.fields.projectile_amount = 1000;
    sentry_cmd.fields.projectile_req_cnt = 1;
    sentry_cmd.fields.hp_req_cnt = 0;
    sentry_cmd.fields.sentry_mode = robot->sentry_mode;
    sentry_cmd.fields.activate_power_rune = 1;
}

static float RemoteChannelToSpeed(int16_t channel, float max_speed)
{
    float normalized = (float)channel / REMOTE_CHANNEL_MAX;
    return normalized * max_speed;
}

static float RemoteDialToWz(int16_t dial)
{
    if (dial >= -REMOTE_DIAL_DEADBAND && dial <= REMOTE_DIAL_DEADBAND)  return 0.0f;
    return -RemoteChannelToSpeed(dial, REMOTE_ROTATEMAX);
}
// 导航和两种遥控共用三轴指令入口，保存本周期指令是否有效。
static void NavigatorControlSet()
{
    static uint8_t offline_logged = 0;
    robot_cmd_t cmd;
    nav_cmd_valid = NavigatorGetCmd(&cmd);
    if (!nav_cmd_valid)
    {
        if (!offline_logged)
            LOGWARNING("[navigator] no valid cmd within %dms, speed forced to zero", NAV_CMD_TIMEOUT_MS);
        offline_logged = 1;
    }
    else
        offline_logged = 0;
    chassis_ctrl_cmd->cmd_frame = CHASSIS_CMD_CHASSIS_FRAME;
    chassis_ctrl_cmd->vx = cmd.speed_vector.vx;
    chassis_ctrl_cmd->vy = cmd.speed_vector.vy;
    chassis_ctrl_cmd->wz = cmd.speed_vector.wz;
    chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
}

#if defined(USE_DUAL_RC)
/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    /* 右拨杆[中]/[上]时底盘处于小陀螺, 云台还会扫头 —— 此时云台指向与底盘朝向不再同步,
     * 平移指令一律按底盘坐标系解释, 不再随云台旋转 */
    uint8_t chassis_frame_cmd = 0;
    // 右[中]，云台底盘使能
    if (switch_is_mid(rc_data[TEMP].rc.switch_right))
    {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
        chassis_frame_cmd = 1;
    }
    // 右[上]，云台底盘使能+扫头
    else if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
        chassis_frame_cmd = 1;
    }
    // 左[中],云台加自瞄
    if (switch_is_mid(rc_data[TEMP].rc.switch_left))
    {
        robot->control_mode = MANUAL_MODE;
    }
    // 左[上],导航+自瞄
    else if (switch_is_up(rc_data[TEMP].rc.switch_left))
    {
        robot->control_mode = NAVIGATOR_MODE;
    }
    else
    {
        robot->control_mode = MANUAL_MODE;
    }

    // 摇杆直接生成 m/s 目标，各轴满量程 2 m/s。
    if (robot->control_mode == MANUAL_MODE) // 手动控制，遥控器控制量
    {
        chassis_ctrl_cmd->vx = RemoteChannelToSpeed(robot->rc_data[TEMP].rc.rocker_l1,
                                                    REMOTE_SPEEDMAX); // 前进为 +x
        chassis_ctrl_cmd->vy = -RemoteChannelToSpeed(robot->rc_data[TEMP].rc.rocker_l_,
                                                     REMOTE_SPEEDMAX); // 左移为 +y
        // 右拨杆[中]/[上]: 底盘系; 其余情况: 云台系(以云台指向为前方, 由底盘任务按云台角度旋转)
        chassis_ctrl_cmd->cmd_frame = chassis_frame_cmd ? CHASSIS_CMD_CHASSIS_FRAME : CHASSIS_CMD_GIMBAL_FRAME;
        chassis_ctrl_cmd->wz = RemoteDialToWz(rc_data[TEMP].rc.dial);
    }
    else if (robot->control_mode == NAVIGATOR_MODE)
    {
        NavigatorControlSet();
    }

    *rc_data_last = *rc_data;
}

#elifdef USE_DUAL_RC_NEW
static void RemoteControlSet()
{
    static float auto_mode_time = 0;
    if (switch_middle(vt13_rc_data->rc.mode_switch) || switch_right(vt13_rc_data->rc.mode_switch))
    {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
        if (abs(vt13_rc_data->rc.dial) > REMOTE_DIAL_DEADBAND)
            chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
    }

    // 新遥控同样直接生成 SI 目标速度。
    if (robot->control_mode == MANUAL_MODE) // 手动控制，遥控器控制量
    {
        vx_initial = RemoteChannelToSpeed(vt13_rc_data->rc.rocker_l1, REMOTE_SPEEDMAX);  // 前进为 +x
        vy_initial = -RemoteChannelToSpeed(vt13_rc_data->rc.rocker_l_, REMOTE_SPEEDMAX); // 左移为 +y
        chassis_ctrl_cmd->wz = RemoteDialToWz(vt13_rc_data->rc.dial);
        auto_mode_time = time;
    }
    else if (robot->control_mode == NAVIGATOR_MODE)
    {
        NavigatorControlSet();
        return;
    }
    // 不要换启动
    chassis_ctrl_cmd->vx = vx_initial;
    chassis_ctrl_cmd->vy = vy_initial;
    // 缓加速
    // if (abs(vx_initial) <= 10000) {
    //   x_speed_time = time;
    //   chassis_ctrl_cmd->vx = vx_initial;
    // }  // 速度绝对值在10000以下输出控制量=输入控制量
    // if (vx_initial > 10000 && chassis_ctrl_cmd->vx <= 60.0f * (float)vt13_rc_data->rc.rocker_l_) {
    //   chassis_ctrl_cmd->vx = 10000 + (time - x_speed_time) * 10000;
    // }
    // if (vx_initial < -10000 && chassis_ctrl_cmd->vx >= 60.0f * (float)vt13_rc_data->rc.rocker_l_) {
    //   chassis_ctrl_cmd->vx = -10000 - (time - x_speed_time) * 10000;
    // }  // 速度绝对值在10000以上输出控制量=10000+10000t(s)
    // if (abs(vy_initial) <= 10000) {
    //   y_speed_time = time;
    //   chassis_ctrl_cmd->vy = vy_initial;
    // }  // 速度绝对值在10000以下输出控制量=输入控制量
    // if (vy_initial > 10000 && chassis_ctrl_cmd->vy <= 60.0f * (float)vt13_rc_data->rc.rocker_l1) {
    //   chassis_ctrl_cmd->vy = 10000 + (time - y_speed_time) * 10000;
    // }
    // if (vy_initial < -10000 && chassis_ctrl_cmd->vy >= 60.0f * (float)vt13_rc_data->rc.rocker_l1) {
    //   chassis_ctrl_cmd->vy = -10000 - (time - y_speed_time) * 10000;
    // }  // 速度绝对值在10000以上输出控制量=10000+10000t(s)
}

#endif

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    // 底盘双板通信离线
    if (!CANCommIsOnline(can_comm_instance))
    {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        LOGERROR("[CMD] Emergency Stop! DualBoardComm Lost");
    }
    else
        LOGINFO("[CMD]DualBoardComm is Online");

#ifdef USE_DUAL_RC

    if (switch_is_down(rc_data_old->Switch_right)) // 底盘失能
    {
        robot->robot_mode = ROBOT_POWER_ON;
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    }
    // 遥控器右侧开关为[上],恢复正常运行

#elifdef USE_DUAL_RC_NEW
    if (switch_left(vt13_rc_data->rc.mode_switch) || vt13_rc_data->button_status.pause_flag == 1) // 底盘失能
    {
        robot->robot_mode = ROBOT_POWER_ON;
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    }
#endif
}

// 手动与导航共用：旋转指令进入死区后，等待实测角速度稳定再锁定当前航向。
static void YawHoldControlSet()
{
    static uint8_t active = 0;
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_POWER_OFF ||
        (robot->control_mode == NAVIGATOR_MODE && !nav_cmd_valid))
    {
        active = 0;
        return;
    }

    chassis_ctrl_cmd->cmd_frame = CHASSIS_CMD_CHASSIS_FRAME;
    if (fabsf(chassis_ctrl_cmd->wz) >= 0.05f || !robot->chassis->chassis_IMU->init)
    {
        active = 0;
        chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
        return;
    }
    chassis_ctrl_cmd->wz = 0.0f;
    if (!active)
    {

        if (robot->chassis->speed == NULL || fabsf(robot->chassis->speed->fused.wz) >= 0.1f)
        {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
            return;
        }
        chassis_ctrl_cmd->yaw_hold_ref = robot->chassis->chassis_IMU->YawTotalAngle;
        PIDClear(&robot->chassis->yaw_hold_pid);
        active = 1;
    }
    chassis_ctrl_cmd->chassis_mode = CHASSIS_HOLD;
}
//todo:模式决策是不是给上位机了，可以删掉吗？
// static void ModeControl()
// {
//
//     if (robot->chassis->chassis_ctrl_cmd.vx == 0 && robot->chassis->chassis_ctrl_cmd.vy == 0)
//     {
//         robot->sentry_mode = OFFENSE_POSE; // 高于50%血或占据堡垒进入进攻姿态
//         if (robot->chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_ROTATE)
//             robot->chassis->chassis_ctrl_cmd.wz -= 1000;
//     }
//     else
//     {
//         robot->sentry_mode = MOBILITY_POSE;
//         if (robot->chassis->chassis_ctrl_cmd.chassis_mode == CHASSIS_ROTATE)
//             robot->chassis->chassis_ctrl_cmd.wz += 1000;
//     }
// }
static void SuperCapControl()
{
    switch (supercap_mode)
    {
    case SAFETY_MODE:
        if (robot->super_cap->cap_msg.cap_v > 18.0f)
            supercap_mode = PASSIVE_MODE;
        robot->chassis->chassis_ctrl_cmd.max_power = 0;
        break;
    case FORCED_CHARGING_MODE:
        if (robot->super_cap->cap_msg.cap_v < 8.0f)
            supercap_mode = SAFETY_MODE;
        if (robot->super_cap->cap_msg.cap_v > 18.0f)
            supercap_mode = PASSIVE_MODE;
        robot->chassis->chassis_ctrl_cmd.max_power =
            (uint16_t)(0.4 * robot->referee_data->GameRobotState.chassis_power_limit);
        break;
    case CHARGING_MODE:
        if (robot->super_cap->cap_msg.cap_v < 10.0f)
            supercap_mode = FORCED_CHARGING_MODE;
        if (robot->super_cap->cap_msg.cap_v > 18.0f)
            supercap_mode = PASSIVE_MODE;
        robot->chassis->chassis_ctrl_cmd.max_power =
            robot->referee_data->GameRobotState.chassis_power_limit -
            (uint16_t)powf((float)robot->referee_data->GameRobotState.chassis_power_limit * 0.04f, 2);
        break;
    case PASSIVE_MODE:
        if (chassis_ctrl_cmd->max_power == 180)
            supercap_mode = ACTIVE_MODE;
        if (robot->super_cap->cap_msg.cap_v < 12.0f)
            supercap_mode = CHARGING_MODE;
        robot->chassis->chassis_ctrl_cmd.max_power = robot->referee_data->GameRobotState.chassis_power_limit;
        break;
    case ACTIVE_MODE:
        if (robot->super_cap->cap_msg.cap_v < 12.0f)
            supercap_mode = CHARGING_MODE;
        if (chassis_ctrl_cmd->max_power != 180)
            supercap_mode = PASSIVE_MODE;
        chassis_ctrl_cmd->max_power = 180;
        break;
    default:
        supercap_mode = SAFETY_MODE;
    }
    SuperCapSendMessage(robot->super_cap, (int16_t)robot->referee_data->GameRobotState.chassis_power_limit,
                        robot->referee_data->PowerHeatData.buffer_energy,
                        robot->referee_data->GameRobotState.power_management_chassis_output);
}
void Navigator_pack()
{
    if (robot->chassis->speed != NULL && robot->chassis->chassis_IMU != NULL && robot->chassis->chassis_IMU->init)
        {
            robot->navigator_send_data->vx = robot->chassis->speed->fused.vx;
            robot->navigator_send_data->vy = robot->chassis->speed->fused.vy;
            robot->navigator_send_data->wz = robot->chassis->speed->fused.wz;
            robot->navigator_send_data->yaw = robot->chassis->chassis_IMU->Yaw;
        }
}
void Chassis_CANCommSend()
{
#ifdef USE_DUAL_RC
    if (can_comm_instance == NULL || rc_data == NULL)
    {
        return;
    }
    // referee_data->projectile_allowance_17mm = robot->referee_data->ProjectileAllowance.projectile_allowance_17mm;
    referee_data->initial_speed = EncodeBulletSpeedToU16(robot->referee_data->ShootData.initial_speed);
    referee_data->shooter_17mm_barrel_heat = robot->referee_data->PowerHeatData.shooter_17mm_barrel_heat;
    referee_data->robot_id = robot->referee_data->GameRobotState.robot_id;
    CANCommSend(can_comm_instance, (void *)referee_data);
#elifdef USE_DUAL_RC_NEW
    if (can_comm_instance == NULL || vt13_rc_data == NULL)
    {
        return;
    }
    // referee_data->projectile_allowance_17mm = robot->referee_data->ProjectileAllowance.projectile_allowance_17mm;
    referee_data->initial_speed = EncodeBulletSpeedToU16(robot->referee_data->ShootData.initial_speed);
    referee_data->shooter_17mm_barrel_heat = robot->referee_data->PowerHeatData.shooter_17mm_barrel_heat;
    referee_data->robot_id = robot->referee_data->GameRobotState.robot_id;
    CANCommSend(can_comm_instance, (void *)referee_data);
#endif
}
// 解析底盘板收到的遥控数据
static void DualBoardCtrlSet()
{
    if (CANCommIsOnline(can_comm_instance))
    {
#ifdef USE_DUAL_RC
        *rc_data_old = *(Send_Data_RC *)CANCommGet(can_comm_instance);
#elifdef USE_DUAL_RC_NEW
        *rc_data_new = *(Send_Data_RC_NEW *)CANCommGet(can_comm_instance);
#endif

#ifdef USE_DUAL_RC
        robot->rc_data[TEMP].rc.rocker_l_ = rc_data_old->Rc_vx; // todo:后面chassis改改把负号去掉
        robot->rc_data[TEMP].rc.rocker_l1 = rc_data_old->Rc_vy;
        robot->rc_data[TEMP].rc.rocker_r_ = rc_data_old->Rotate_speed;
        robot->rc_data[TEMP].rc.dial = rc_data_old->Spin_speed;
        robot->rc_data[TEMP].rc.switch_right = rc_data_old->Switch_right;
        robot->rc_data[TEMP].rc.switch_left = rc_data_old->Switch_left;

        if (switch_is_mid(rc_data_old->Switch_right))
        {
            // gimbal_ctrl_cmd->gimbal_mode = GIMBAL_ON;
            if (abs(rc_data_old->Spin_speed) > REMOTE_DIAL_DEADBAND)
            {
                chassis_ctrl_cmd->chassis_mode = CHASSIS_ROTATE;
            }
            else
                chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
        }

#elifdef USE_DUAL_RC_NEW
        vt13_rc_data->rc.rocker_l_ = rc_data_new->Rc_vx;
        vt13_rc_data->rc.rocker_l1 = rc_data_new->Rc_vy;
        vt13_rc_data->rc.rocker_r_ = rc_data_new->Rotate_speed;
        vt13_rc_data->rc.dial = rc_data_new->Spin_speed;
        vt13_rc_data->rc.mode_switch = rc_data_new->Mode_switch;
        robot->control_mode = rc_data_new->Control_mode;
        vt13_rc_data->button_status.pause_flag = rc_data_new->Pause_flag;
#endif
    }
}


void RobotInit()
{
    // 要在云台和底盘任务开始之前完成该任务的初始化
    vTaskDelay(CAN_COMM_TASK_INIT_TIME);
    // 初始化CAN接收
    can_comm_instance = CANCommInit(&comm_config);
    robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));
    referee_data = (Referee_Data *)zmalloc(sizeof(Referee_Data));
#ifdef USE_DUAL_RC
    // 使用旧遥控器
    rc_data_old = (Send_Data_RC *)zmalloc(sizeof(Send_Data_RC));
    rc_data_last = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));
    robot->rc_data = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));
    *rc_data_last = *robot->rc_data; // 记录上一次遥控器的状态
    rc_data = robot->rc_data;
#elif defined(USE_DUAL_RC_NEW)
    // 使用新VT13遥控器
    robot->vt13_rc_data = (VT13_RC_t *)zmalloc(sizeof(VT13_RC_t));
    vt13_rc_data = robot->vt13_rc_data;
    rc_data_new = (Send_Data_RC_NEW *)zmalloc(sizeof(Send_Data_RC_NEW));
#endif

    // robot->vision_recv_data = VisionInit(&gimbal_init_config.imu_init_config);
    robot->navigator_data = navigator_init(&huart1); // 导航串口: USART1, 115200 8N1

    robot->referee_data = (referee_info_t *)zmalloc(sizeof(referee_info_t));
    robot->sentry_mode = 1;

    robot->super_cap = SuperCapInit(&super_cap_config);

    robot->chassis = ChassisInit(&chassis_init_config);
    
    robot->navigator_send_data = (navigator_send_t *)zmalloc(sizeof(navigator_send_t));
    // 初始化控制命令指针
    chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发频率) */
void RobotCMDTask()
{
    static float last_rc_dualboard_time = 0.0f;
    static uint8_t rc_dualboard_first_run = 1;
    time = DWT_GetTimeline_s();
    // 双板数据按100Hz更新，其他安全逻辑维持高频
    if (rc_dualboard_first_run || (time - last_rc_dualboard_time) >= 0.012f)
    {
        rc_dualboard_first_run = 0;
        last_rc_dualboard_time = time;
        Chassis_CANCommSend();
        // SentryRefereeSend();
    }
    DualBoardCtrlSet();
    CalcOffsetAngle();
    RemoteControlSet();
    // MouseKeySet();
    EmergencyHandler();  // 处理模块离线和遥控器急停等紧急情况
    YawHoldControlSet(); // 三轴指令和急停处理之后，统一选择旋转或航向保持。
}

void RobotTask()
{
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    GimbalTask();
#endif
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    RobotCMDTask();
    // SuperCapControl();
    chassis_ctrl_cmd->max_power = 120;
    // ModeControl();
    ChassisTask();
    Navigator_pack();
    navigator_send(&huart1, robot->referee_data, robot->navigator_send_data); // 每 10ms 轮流反馈 0x0001 / 0x000B / 0x0010
    // SuperCapSendMessage(robot->super_cap,
    //     (int16_t)robot->referee_data->GameRobotState.chassis_power_limit,
    //     robot->referee_data->PowerHeatData.buffer_energy,
    //     robot->referee_data->GameRobotState.power_management_chassis_output);
#endif
}
