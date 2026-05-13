/* Private includes ----------------------------------------------------------*/
#include "robot.h"
#include "bsp_gpio.h"
#include "cmsis_os.h"
#include "general_def.h"
#include "half_auto.h"
#include "ins_task.h"
#include "robot_config.h"
#include "selfcontrol.h"
#include "stdbool.h"
#include "stdlib.h"
#include "string.h"
#include "user_lib.h"
#include "new_RC_VT13.h"

/* Private define ------------------------------------------------------------*/
// 0.3s消抖阈值 (基于2ms的任务周期: 300ms / 2ms = 150)
#define SWITCH_STABLE_TICKS 150
/* ==================== 图传控制灵敏度 ==================== */
// 鼠标移动折算成图传指令的比例系数 (越大转越快)
#define VIDEO_MOUSE_YAW_SENS 0.155f
#define VIDEO_MOUSE_PITCH_SENS 0.155f
/* Intermediate variables calculated by private functions */
static RobotInstance *robot;
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;
static Grab_Ctrl_Cmd_s *grab_ctrl_cmd;

static VideoGimbal_Ctrl_Cmd_s *video_gimbal_ctrl_cmd;

// 【修改2】将数据类型换为新版遥控器结构体
static VT13_RC_t *rc_data;
// static RC_ctrl_t *rc_data_last; // 【移除】新版遥控器内部自带 LAST 数组，无需外部维护

static float set_angle = 0;
static int save_point_trigger = 0;
static Grab_Param_s grab_param;
static GrabControlMode_e grab_control_mode = GRAB_CONTROL_KEYBOARD; // 默认为键鼠控制

float custom_trajectory[50][9];
uint16_t custom_traj_length = 0;
/* Private function prototypes -----------------------------------------------*/
static GPIOInstance *gpio_5V_EN;
static GPIO_Init_Config_s gpio_init_config_5v = {
    .GPIO_Pin = POWER_5V_Pin,
    .GPIOx = POWER_5V_GPIO_Port,
    .pin_state = GPIO_PIN_SET,
};
static void Record_Current_Waypoint(void);
static void RemoteControlSet();
static void MouseKeySet();
static void EmergencyHandler();
static void ProcessCustomControllerData();
static void CalcOffsetAngle();
static void SendArmMotorDataTask(void); // 发送机械臂电机数据任务
void RobotInit();
void RobotCMDTask();
void RobotTask();

/* Private user code ---------------------------------------------------------*/
void RobotInit()
{
    robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));
    robot->referee_data = RefereeInit(&huart1); // 裁判系统初始化
#ifdef STM32F4
    robot->rc_data = VT13RemoteInit(&huart3); // 【修改3】调用新版遥控器初始化接口
#elifdef STM32H7
    robot->rc_data = VT13RemoteInit(&huart5);
    robot->self_control = SelfControlInit(&huart7); // 初始化自定义控制器
#endif

    robot->ins_data = INS_Init(&imu_init_config);
    robot->grab = GrabInit(&grab_init_config);
    robot->video_gimbal = VideoGimbalInit(&video_gimbal_init_config);

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    robot->chassis = ChassisInit(&chassis_init_config);
#endif

    // 初始化控制命令指针
    chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
    chassis_ctrl_cmd->max_power = 120; // 随便给一个初始功率，后面应该要从裁判系统获取
    grab_ctrl_cmd = &robot->grab->grab_ctrl_cmd;
    video_gimbal_ctrl_cmd = &robot->video_gimbal->ctrl_cmd;

    grab_param = grab_init_config.Grab_param;

    rc_data = robot->rc_data;
    gpio_5V_EN = GPIORegister(&gpio_init_config_5v);
    GPIOSet(gpio_5V_EN);

    // 初始化 UI 重置标志位
    robot->ui_reset_flag = 0;
}

void RobotTask()
{
#if defined(ONE_BOARD) || defined(GIMBAL_BOARD)
    RobotCMDTask();
#endif

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    ChassisTask();
#endif

#if defined(ONE_BOARD)
    GrabTask();
    VideoGimbalTask();
#endif

    // 发送机械臂电机数据给自定义控制器 (10Hz)
    SendArmMotorDataTask();
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    grab_ctrl_cmd->is_climb_mode = (robot->robot_mode == ROBOT_CLIMB_MODE) ? 1 : 0;

    if (grab_control_mode == GRAB_CONTROL_CUSTOM)
    {
        // 🛡️ 标定进行中时，屏蔽自定义控制器对 grab_ctrl_cmd 的覆写，防止疯转
        if (robot->grab->actuator->wrist_cali_obj.state == CALI_DONE &&
            robot->grab->arm->extend_cali_obj.state == CALI_DONE)
        {
            ProcessCustomControllerData();
        }
    }
    else if (grab_control_mode == GRAB_CONTROL_HALF_AUTO)
    {
        // 【修改4】使用新版遥控器内部的 TEMP 和 LAST 鼠标历史帧数据
        Half_auto_update(grab_ctrl_cmd, chassis_ctrl_cmd,
                         rc_data->mouse_key[TEMP].mouse.press_l, rc_data->mouse_key[LAST].mouse.press_l,
                         rc_data->mouse_key[TEMP].mouse.press_r, rc_data->mouse_key[LAST].mouse.press_r);
    }
    CalcOffsetAngle();
    RemoteControlSet();
    MouseKeySet();
    Record_Current_Waypoint();
    EmergencyHandler();
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    if (rc_data == NULL)
        return;

    // 【修改5】将 rc_data[TEMP].rc 换为 rc_data->rc
    // 屏蔽遥控器摇杆输入干扰
    if (rc_data->rc.dial != 0 || rc_data->rc.rocker_l1 != 0 || rc_data->rc.rocker_l_ != 0 ||
        rc_data->rc.rocker_r1 != 0 || rc_data->rc.rocker_r_ != 0)
    {
        return; // 有摇杆输入时不进行键鼠控制
    }

    // 1. 把爬楼状态机变量提升到函数开头，方便全局复位
    static uint8_t keyboard_climb_state = CHASSIS_CLIMB_IDLE;
    // 记录上一次的大模式，用于边沿检测
    static uint8_t last_robot_mode = ROBOT_POWER_OFF;

    // ================= 1. 大模式切换 (按 G 键循环切换) =================
    // 【修改6】rc_data[TEMP].key 换为 rc_data->key，下同，全部替换
    switch (rc_data->key_count[KEY_PRESS_NORMAL][KEY_G] % 4)
    {
    case 0:
        robot->robot_mode = ROBOT_POWER_ON;
        break;
    case 1:
        robot->robot_mode = ROBOT_EXCHANGE_MODE;
        break;
    case 2:
        robot->robot_mode = ROBOT_CLIMB_MODE;
        break;
    case 3:
        robot->robot_mode = ROBOT_BUMPY_MODE; // 🌟 烂路模式
        break;
    }

    if (robot->robot_mode != last_robot_mode)
    {
        // 🌟 新增：只要车体大模式发生了切换，直接强制清零一切半自动的残留步数！
        Half_auto_reset();

        if (last_robot_mode == ROBOT_CLIMB_MODE || last_robot_mode == ROBOT_BUMPY_MODE)
        {
            keyboard_climb_state = CHASSIS_CLIMB_IDLE;
        }
        if (last_robot_mode == ROBOT_EXCHANGE_MODE)
        {
            chassis_ctrl_cmd->lift_ratio = 0.0f;
        }
        last_robot_mode = robot->robot_mode;
    }

    if (robot->robot_mode != ROBOT_POWER_OFF && robot->robot_mode != ROBOT_EMERGENCY_STOP)
    {
        grab_ctrl_cmd->grab_mode = GRAB_POWER_ON;
        video_gimbal_ctrl_cmd->power = VIDEO_POWER_ON;

        // ================= 2. 机械臂控制权切换 =================
        if (robot->robot_mode == ROBOT_EXCHANGE_MODE || robot->robot_mode == ROBOT_CLIMB_MODE)
        {
            uint8_t curr_f_only = rc_data->key[KEY_PRESS_NORMAL].f;
            static uint8_t last_f_only = 0;

            uint8_t curr_ctrl_f = rc_data->key[KEY_PRESS_WITH_CTRL].f;
            static uint8_t last_ctrl_f = 0;

            if (curr_ctrl_f && !last_ctrl_f)
            {
                grab_control_mode = GRAB_CONTROL_HALF_AUTO;
            }

            if (curr_f_only && !last_f_only)
            {
                if (grab_control_mode == GRAB_CONTROL_KEYBOARD)
                    grab_control_mode = GRAB_CONTROL_CUSTOM;
                else
                    grab_control_mode = GRAB_CONTROL_KEYBOARD; // 优先保底
            }

            last_f_only = curr_f_only;
            last_ctrl_f = curr_ctrl_f;
        }
        else
        {
            grab_control_mode = GRAB_CONTROL_KEYBOARD;
        }

        // ================= 3. 底盘平移 (WASD 全局生效) =================
        float speed_buff = 20000;
        uint8_t r_state = rc_data->key_count[KEY_PRESS_NORMAL][KEY_R] % 2;

        if (r_state == 1)
        {
            chassis_ctrl_cmd->vx = -(float)(rc_data->key[KEY_PRESS_NORMAL].w - rc_data->key[KEY_PRESS_NORMAL].s) * speed_buff;
            chassis_ctrl_cmd->vy =  (float)(rc_data->key[KEY_PRESS_NORMAL].d - rc_data->key[KEY_PRESS_NORMAL].a) * speed_buff;
        }
        else
        {
            chassis_ctrl_cmd->vx =  (float)(rc_data->key[KEY_PRESS_NORMAL].d - rc_data->key[KEY_PRESS_NORMAL].a) * speed_buff;
            chassis_ctrl_cmd->vy =  (float)(rc_data->key[KEY_PRESS_NORMAL].w - rc_data->key[KEY_PRESS_NORMAL].s) * speed_buff;
        }

        // ================= 新增：Shift+R 图传云台一键回正 =================
        static uint8_t last_shift_r = 0;
        uint8_t current_shift_r = rc_data->key[KEY_PRESS_WITH_SHIFT].r;

        if (current_shift_r && !last_shift_r)
        {
            uint8_t r_state_inner = rc_data->key_count[KEY_PRESS_NORMAL][KEY_R] % 2;
            float current_video_yaw = robot->video_gimbal->Video_yaw;

            if (r_state_inner == 1)
                robot->video_gimbal->Video_yaw = roundf((current_video_yaw + 90.0f) / 360.0f) * 360.0f - 90.0f;
            else
                robot->video_gimbal->Video_yaw = roundf(current_video_yaw / 360.0f) * 360.0f;
        }
        last_shift_r = current_shift_r;

        // 旋转量
        float angle_buff = 0.15f;
        float angle_rapid_buff = 0.4f;
        if (robot->robot_mode == ROBOT_EXCHANGE_MODE)
        {
            if (chassis_ctrl_cmd->lift_ratio - 0.1f < 0.01f)
            {
                set_angle += (float)((rc_data->key[KEY_PRESS_WITH_SHIFT].q - rc_data->key[KEY_PRESS_WITH_SHIFT].e) * angle_buff +
                                     (rc_data->key[KEY_PRESS_WITH_SHIFT].a - rc_data->key[KEY_PRESS_WITH_SHIFT].d) * angle_rapid_buff);
            }
        }
        else if (robot->robot_mode == ROBOT_CLIMB_MODE || robot->robot_mode == ROBOT_BUMPY_MODE)
        {
            if (chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_ALL_RETRACT ||
                chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_FRONT_RETRACT_REAR_HALF ||
                chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_FRONT_RETRACT)
            {
                set_angle += (float)((rc_data->key[KEY_PRESS_WITH_SHIFT].q - rc_data->key[KEY_PRESS_WITH_SHIFT].e) * angle_buff +
                                     (rc_data->key[KEY_PRESS_WITH_SHIFT].a - rc_data->key[KEY_PRESS_WITH_SHIFT].d) * angle_rapid_buff);
            }
        }
        chassis_ctrl_cmd->robot_mode = robot->robot_mode;

        // ================= 4. 姿态复用控制 (Q, E, R) =================
        if (robot->robot_mode == ROBOT_EXCHANGE_MODE)
        {
            float step_size = 1.0f / (12.0f * 200.0f);
            chassis_ctrl_cmd->lift_ratio += (float)(rc_data->key[KEY_PRESS_NORMAL].q - rc_data->key[KEY_PRESS_NORMAL].e) * step_size;

            if (chassis_ctrl_cmd->lift_ratio < 0.0f) chassis_ctrl_cmd->lift_ratio = 0.0f;
            if (chassis_ctrl_cmd->lift_ratio > 1.0f) chassis_ctrl_cmd->lift_ratio = 1.0f;
        }
        else if (robot->robot_mode == ROBOT_CLIMB_MODE || robot->robot_mode == ROBOT_BUMPY_MODE)
        {
            uint8_t key_q = rc_data->key[KEY_PRESS_NORMAL].q;
            uint8_t key_e = rc_data->key[KEY_PRESS_NORMAL].e;

            static uint8_t last_raw_state = 0;
            static uint8_t stable_cnt = 0;
            uint8_t current_state = (key_q << 1) | key_e;

            if (current_state == last_raw_state)
            {
                if (stable_cnt < 15) stable_cnt++;
                if (stable_cnt == 10)
                {
                    if (current_state == 3)
                        keyboard_climb_state = CHASSIS_CLIMB_BOTH_EXTEND;
                    else if (current_state == 2)
                    {
                        if (keyboard_climb_state != CHASSIS_CLIMB_FRONT_RETRACT_REAR_HALF &&
                            keyboard_climb_state != CHASSIS_CLIMB_FRONT_RETRACT)
                        {
                            keyboard_climb_state = CHASSIS_CLIMB_FRONT_RETRACT_REAR_HALF;
                        }
                        else if (keyboard_climb_state == CHASSIS_CLIMB_FRONT_RETRACT_REAR_HALF)
                        {
                            keyboard_climb_state = CHASSIS_CLIMB_FRONT_RETRACT;
                        }
                    }
                    else if (current_state == 1)
                        keyboard_climb_state = CHASSIS_CLIMB_ALL_RETRACT;
                }
            }
            else
            {
                stable_cnt = 0;
            }
            last_raw_state = current_state;

            if (keyboard_climb_state != CHASSIS_CLIMB_IDLE)
            {
                chassis_ctrl_cmd->chassis_mode = keyboard_climb_state;
            }
        }

        // 机械臂微调
        if (robot->robot_mode == ROBOT_EXCHANGE_MODE || robot->robot_mode == ROBOT_CLIMB_MODE)
        {
            if (grab_control_mode == GRAB_CONTROL_KEYBOARD || grab_control_mode == GRAB_CONTROL_CUSTOM)
            {
                grab_ctrl_cmd->arm_lift += (float)(rc_data->key[KEY_PRESS_WITH_SHIFT].w - rc_data->key[KEY_PRESS_WITH_SHIFT].s) * grab_param.arm_lift_sens_keyboard;
                float extend_speed = 4.0f;
                grab_ctrl_cmd->arm_extend += (float)(rc_data->key[KEY_PRESS_WITH_CTRL_SHIFT].v - rc_data->key[KEY_PRESS_WITH_CTRL_SHIFT].b) * extend_speed;
            }

            if (grab_control_mode == GRAB_CONTROL_KEYBOARD)
            {
                float arm_speed = 0.08f;
                grab_ctrl_cmd->base_joint  += (float)(rc_data->key[KEY_PRESS_WITH_CTRL_SHIFT].q - rc_data->key[KEY_PRESS_WITH_CTRL_SHIFT].w) * arm_speed;
                grab_ctrl_cmd->elbow_roll  += (float)(rc_data->key[KEY_PRESS_WITH_CTRL_SHIFT].e - rc_data->key[KEY_PRESS_WITH_CTRL_SHIFT].r) * arm_speed;
                grab_ctrl_cmd->elbow_pitch += (float)(rc_data->key[KEY_PRESS_WITH_CTRL_SHIFT].a - rc_data->key[KEY_PRESS_WITH_CTRL_SHIFT].s) * arm_speed;
                grab_ctrl_cmd->wrist_pitch += (float)(rc_data->key[KEY_PRESS_WITH_CTRL_SHIFT].d - rc_data->key[KEY_PRESS_WITH_CTRL_SHIFT].f) * arm_speed;
                grab_ctrl_cmd->wrist_roll  += (float)(rc_data->key[KEY_PRESS_WITH_CTRL_SHIFT].z - rc_data->key[KEY_PRESS_WITH_CTRL_SHIFT].x) * arm_speed;
            }
        }
    }

    // ================= 5. 夹爪控制 =================
    uint32_t current_c_count = rc_data->key_count[KEY_PRESS_NORMAL][KEY_C];

    if (grab_control_mode == GRAB_CONTROL_KEYBOARD || grab_control_mode == GRAB_CONTROL_CUSTOM)
    {
        if (current_c_count % 2 == 1)
            grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        else
            grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
    }
    else if (grab_control_mode == GRAB_CONTROL_HALF_AUTO)
    {
        if (grab_ctrl_cmd->gripper_state == GRIPPER_CLOSE)
            rc_data->key_count[KEY_PRESS_WITH_CTRL_SHIFT][KEY_C] = 1;
        else
            rc_data->key_count[KEY_PRESS_WITH_CTRL_SHIFT][KEY_C] = 0;
    }

    // ================= 6. 图传 Yaw/Pitch 控制 (修正鼠标数据访问层级) =================
    video_gimbal_ctrl_cmd->video_pitch =
        (float)(rc_data->key[KEY_PRESS_NORMAL].x - rc_data->key[KEY_PRESS_NORMAL].z) -
        (float)rc_data->mouse_key[TEMP].mouse.y * VIDEO_MOUSE_PITCH_SENS;

    video_gimbal_ctrl_cmd->video_yaw =
        (float)(rc_data->key[KEY_PRESS_NORMAL].b - rc_data->key[KEY_PRESS_NORMAL].v) +
        (float)rc_data->mouse_key[TEMP].mouse.x * VIDEO_MOUSE_YAW_SENS;

    // ================= 7. 标定与 UI =================
    if (rc_data->key[KEY_PRESS_WITH_CTRL].q)
        grab_ctrl_cmd->wrist_roll_cali = 1;
    else if (rc_data->key[KEY_PRESS_WITH_CTRL].e)
        grab_ctrl_cmd->wrist_pitch_cali = 1;
    else if (rc_data->key[KEY_PRESS_WITH_CTRL].v)
        video_gimbal_ctrl_cmd->video_cali = 1;

    static uint8_t last_ctrl_b = 0;
    uint8_t curr_ctrl_b = rc_data->key[KEY_PRESS_WITH_CTRL].b;

    if (curr_ctrl_b && !last_ctrl_b)
    {
        robot->ui_reset_flag = 1;
    }
    last_ctrl_b = curr_ctrl_b;
}

static void EmergencyHandler()
{
    // 【修改7】VT13没有拨杆急停，这里修改为通过自带的 pause 按键急停
    if (rc_data->button_status.pause_flag || !VT13RemoteIsOnline())
    {
        robot->robot_mode = ROBOT_EMERGENCY_STOP;
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        grab_ctrl_cmd->grab_mode = GRAB_POWER_OFF;
        video_gimbal_ctrl_cmd->power = VIDEO_POWER_OFF;
        LOGINFO("[CMD] emergency stop!");
    }
}

static void RemoteControlSet()
{
    if (!robot->chassis->cali_state.all_cali_done)
    {
        if (VT13RemoteIsOnline())
            chassis_ctrl_cmd->chassis_mode = CHASSIS_CALIBRATING;

        // VT13的模式拨杆在最低档时断电
        if (rc_data->rc.mode_switch == 2)
            chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        return;
    }

    bool is_keyboard_climb = (robot->robot_mode == ROBOT_CLIMB_MODE || robot->robot_mode == ROBOT_BUMPY_MODE);

    // 【修改8】适配 VT13 的单拨杆（档位：1上，3中，2下）
    if (rc_data->rc.mode_switch == 3) // 中档：底盘跟随
    {
        if (!is_keyboard_climb)
        {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
        }

        if (abs(rc_data->rc.dial) > 20)
        {
            chassis_ctrl_cmd->wz = 0;
            set_angle += (rc_data->rc.dial - 20) * 0.0001;
        }
    }
    else if (rc_data->rc.mode_switch == 2) // 下档：底盘下电
    {
        if (!is_keyboard_climb)
        {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        }
    }
    else if (rc_data->rc.mode_switch == 1) // 上档
    {
        // 【说明】VT13 只有一个左拨杆，没有原先的第二根拨杆来控制爬楼台阶。
        // 所以原先用 RC 控制爬楼动作的逻辑被移除，现在上台阶请完全依靠键盘的 Q 和 E 进行姿态调节！
        if (!is_keyboard_climb)
        {
            if (chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_IDLE &&
                chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_BOTH_EXTEND &&
                chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_FRONT_RETRACT_REAR_HALF &&
                chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_FRONT_RETRACT &&
                chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_ALL_RETRACT)
            {
                chassis_ctrl_cmd->chassis_mode = CHASSIS_CLIMB_IDLE;
            }
        }
    }

    // 夹爪控制 rocker_r1
    if (rc_data->rc.rocker_r1 > 300)
    {
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        if (rc_data->key_count[KEY_PRESS_WITH_CTRL_SHIFT][KEY_C] % 2 == 0)
        {
            rc_data->key_count[KEY_PRESS_WITH_CTRL_SHIFT][KEY_C]++;
        }
    }
    else if (rc_data->rc.rocker_r1 < -300)
    {
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        if (rc_data->key_count[KEY_PRESS_WITH_CTRL_SHIFT][KEY_C] % 2 == 1)
        {
            rc_data->key_count[KEY_PRESS_WITH_CTRL_SHIFT][KEY_C]++;
        }
    }

    chassis_ctrl_cmd->vx = 60.0f * (float)rc_data->rc.rocker_l_;
    chassis_ctrl_cmd->vy = 60.0f * (float)rc_data->rc.rocker_l1;

    if (abs(rc_data->rc.dial) > 20)
    {
        if (chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_ALL_RETRACT ||
            chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_FRONT_RETRACT_REAR_HALF ||
            chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_FRONT_RETRACT)
        {
            set_angle += (rc_data->rc.dial - 20) * 0.0001;
        }
        chassis_ctrl_cmd->wz = 0;
    }

    chassis_ctrl_cmd->wz = 0;
}

static void ProcessCustomControllerData()
{
    if (robot->self_control != NULL)
    {
        if (robot->grab != NULL && grab_ctrl_cmd != NULL)
        {
            grab_ctrl_cmd->base_joint = SelfControlGetMotorAngle(robot->self_control, 0);
            grab_ctrl_cmd->elbow_roll = SelfControlGetMotorAngle(robot->self_control, 1);
            grab_ctrl_cmd->elbow_pitch = SelfControlGetMotorAngle(robot->self_control, 2);
            grab_ctrl_cmd->wrist_pitch = SelfControlGetMotorAngle(robot->self_control, 3);
            grab_ctrl_cmd->wrist_roll = SelfControlGetMotorAngle(robot->self_control, 4);
        }
    }
}

static void CalcOffsetAngle()
{
    chassis_ctrl_cmd->offset_angle = set_angle - robot->ins_data->YawTotalAngle;
}

GrabControlMode_e GetGrabControlMode(void)
{
    return grab_control_mode;
}

RobotInstance *RobotGet(void)
{
    return robot;
}

static void Record_Current_Waypoint(void)
{
    if (save_point_trigger == 1)
    {
        if (custom_traj_length < 50)
        {
            custom_trajectory[custom_traj_length][0] = grab_ctrl_cmd->base_joint;
            custom_trajectory[custom_traj_length][1] = grab_ctrl_cmd->elbow_roll;
            custom_trajectory[custom_traj_length][2] = grab_ctrl_cmd->elbow_pitch;
            custom_trajectory[custom_traj_length][3] = grab_ctrl_cmd->wrist_pitch;
            custom_trajectory[custom_traj_length][4] = grab_ctrl_cmd->wrist_roll;
            custom_trajectory[custom_traj_length][5] = (grab_ctrl_cmd->gripper_state == GRIPPER_CLOSE) ? 1.0f : 0.0f;
            custom_trajectory[custom_traj_length][6] = chassis_ctrl_cmd->lift_ratio;
            custom_trajectory[custom_traj_length][7] = grab_ctrl_cmd->arm_extend;
            custom_trajectory[custom_traj_length][8] = grab_ctrl_cmd->arm_lift;

            custom_traj_length++;
            save_point_trigger = 0;
        }
    }
}

static void SendArmMotorDataTask(void)
{
    static uint32_t last_send_time = 0;
    uint32_t current_time = HAL_GetTick();

    if ((current_time - last_send_time) < 100) return;

    float motor_angles[5] = {0.0f};
    if (robot->grab != NULL && robot->grab->arm != NULL && robot->grab->actuator != NULL)
    {
        if (grab_control_mode == GRAB_CONTROL_HALF_AUTO)
        {
            motor_angles[0] = robot->grab->grab_ctrl_cmd.base_joint;
            motor_angles[1] = robot->grab->grab_ctrl_cmd.elbow_roll;
            motor_angles[2] = robot->grab->grab_ctrl_cmd.elbow_pitch;
            motor_angles[3] = robot->grab->grab_ctrl_cmd.wrist_pitch;
            motor_angles[4] = robot->grab->grab_ctrl_cmd.wrist_roll;
        }
        else
        {
            motor_angles[0] = robot->grab->grab_measure.base_joint;
            motor_angles[1] = robot->grab->grab_measure.elbow_roll;
            motor_angles[2] = robot->grab->grab_measure.elbow_pitch;
            motor_angles[3] = robot->grab->grab_measure.wrist_pitch;
            motor_angles[4] = robot->grab->grab_measure.wrist_roll;
        }
    }

    uint8_t control_mode = (uint8_t)grab_control_mode;

    if (robot->self_control != NULL && robot->self_control->usart_instance != NULL)
    {
        SelfControl_SendMotorDataToCustom(motor_angles, control_mode, robot->self_control->usart_instance);
    }

    last_send_time = current_time;
}