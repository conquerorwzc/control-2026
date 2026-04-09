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
static Gantry_Ctrl_Cmd_s *gantry_ctrl_cmd; // 【新增】龙门架控制命令指针
static VideoGimbal_Ctrl_Cmd_s *video_gimbal_ctrl_cmd;
static RC_ctrl_t *rc_data;
static RC_ctrl_t *rc_data_last; // 遥控器数据,初始化时返回
static float set_angle = 0;
static int save_point_trigger = 0;
static float angle = 0;
static float target_angle = 0;
static int mouse_l_count = 0;
static uint32_t current_selfcontrol_gripper = 0;
static uint32_t last_selfcontrol_gripper = 0;
static Gantry_Param_s gantry_param;
static Grab_Param_s grab_param;
static GrabControlMode_e grab_control_mode = GRAB_CONTROL_KEYBOARD; // 默认为键鼠控制

float custom_trajectory[50][7];
uint16_t custom_traj_length = 0; // 记录当前步数
/* Private function prototypes -----------------------------------------------*/
static void Gantry_Limit(Gantry_Ctrl_Cmd_s *gantry_ctrl_cmd, const Gantry_Param_s *gantry_param);
static void Grab_Limit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, const Gantry_Param_s *gantry_param);
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
void RobotInit();
void RobotCMDTask();
void RobotTask();

/* Private user code ---------------------------------------------------------*/
void RobotInit()
{
    robot = (RobotInstance *)zmalloc(sizeof(RobotInstance));
    robot->referee_data = RefereeInit(&huart1); // 裁判系统初始化
#ifdef STM32F4
    robot->rc_data = RemoteControlInit(&huart3); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#elifdef STM32H7
    robot->rc_data = RemoteControlInit(&huart5); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    robot->self_control = SelfControlInit(&huart7); // 初始化自定义控制器
#endif

    rc_data_last = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));
    *rc_data_last = *robot->rc_data; // 记录上一次遥控器的状态
    robot->ins_data = INS_Init(&imu_init_config);
    // robot->gantry = GantryInit(&gantry_init_config);
    robot->grab = GrabInit(&grab_init_config);
    robot->video_gimbal = VideoGimbalInit(&video_gimbal_init_config);

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    robot->chassis = ChassisInit(&chassis_init_config);
#endif

    // 初始化控制命令指针
    chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
    chassis_ctrl_cmd->max_power = 80; // 随便给一个初始功率，后面应该要从裁判系统获取
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

    // 新增: 龙门架控制逻辑 (GantryTask)
#if defined(ONE_BOARD)
    // GantryTask();
    GrabTask();
    VideoGimbalTask();
#endif
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    if (grab_control_mode == GRAB_CONTROL_CUSTOM)
    {
        ProcessCustomControllerData();
    }
    else if (grab_control_mode == GRAB_CONTROL_HALF_AUTO)
    {
        Half_auto_update(grab_ctrl_cmd, chassis_ctrl_cmd, rc_data->mouse.press_l, rc_data_last->mouse.press_l,
                         rc_data->mouse.press_r, rc_data_last->mouse.press_r);
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

    // 屏蔽遥控器摇杆输入干扰
    if (rc_data[TEMP].rc.dial != 0 || rc_data[TEMP].rc.rocker_l1 != 0 || rc_data[TEMP].rc.rocker_l_ != 0 ||
        rc_data[TEMP].rc.rocker_r1 != 0 || rc_data[TEMP].rc.rocker_r_ != 0)
    {
        return; // 有摇杆输入时不进行键鼠控制
    }

    // ================= 1. 大模式切换 (按 G 键循环切换) =================
    switch (rc_data[TEMP].key_count[KEY_PRESS_NORMAL][KEY_G] % 2)
    {
    case 0:
        robot->robot_mode = ROBOT_POWER_ON;
        break;
    case 1:
        robot->robot_mode = ROBOT_EXCHANGE_MODE;
        break;
        // case 2:
        //     robot->robot_mode = ROBOT_CLIMB_MODE;
        //     break;
    }

    if (robot->robot_mode != ROBOT_POWER_OFF && robot->robot_mode != ROBOT_EMERGENCY_STOP)
    {
        grab_ctrl_cmd->grab_mode = GRAB_POWER_ON;
        video_gimbal_ctrl_cmd->power = VIDEO_POWER_ON;

        // ================= 2. 机械臂控制权切换 (按 F 键) =================
        if (robot->robot_mode == ROBOT_EXCHANGE_MODE)
        {
            switch (rc_data[TEMP].key_count[KEY_PRESS_NORMAL][KEY_F] % 3)
            {
            case 0:
                grab_control_mode = GRAB_CONTROL_KEYBOARD;
                break;
            case 1:
                grab_control_mode = GRAB_CONTROL_HALF_AUTO;
                break;
            case 2:
                grab_control_mode = GRAB_CONTROL_CUSTOM;
                break;
            }
        }
        else
        {
            grab_control_mode = GRAB_CONTROL_KEYBOARD;
            rc_data[TEMP].key_count[KEY_PRESS_NORMAL][KEY_F] = 0;
        }

        // ================= 3. 底盘平移 (WASD 全局生效) =================
        float speed_buff = 20000;

        // 提取 R 键的状态：0 为正向，1 为反向
        uint8_t r_state = rc_data[TEMP].key_count[KEY_PRESS_NORMAL][KEY_R] % 2;

        if (r_state == 1)
        {
            // 反向平移
            chassis_ctrl_cmd->vx =
                -(float)(rc_data[TEMP].key[KEY_PRESS_NORMAL].w - rc_data[TEMP].key[KEY_PRESS_NORMAL].s) * speed_buff;
            chassis_ctrl_cmd->vy =
                (float)(rc_data[TEMP].key[KEY_PRESS_NORMAL].d - rc_data[TEMP].key[KEY_PRESS_NORMAL].a) * speed_buff;
        }
        else
        {
            // 正向平移
            chassis_ctrl_cmd->vx =
                (float)(rc_data[TEMP].key[KEY_PRESS_NORMAL].d - rc_data[TEMP].key[KEY_PRESS_NORMAL].a) * speed_buff;
            chassis_ctrl_cmd->vy =
                (float)(rc_data[TEMP].key[KEY_PRESS_NORMAL].w - rc_data[TEMP].key[KEY_PRESS_NORMAL].s) * speed_buff;
        }

        // ================= 新增：Shift+R 图传云台一键回正 =================
        static uint8_t last_shift_r = 0;
        uint8_t current_shift_r = rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].r;

        // 边沿检测，按下的瞬间触发
        if (current_shift_r && !last_shift_r)
        {
            // 提取 R 键的状态：0 为正向，1 为侧向
            uint8_t r_state = rc_data[TEMP].key_count[KEY_PRESS_NORMAL][KEY_R] % 2;

            // 获取图传云台当前的累积相对角度
            float current_video_yaw = robot->video_gimbal->Video_yaw;

            if (r_state == 1)
            {
                // R键侧向状态：图传云台就近转到 90 度
                robot->video_gimbal->Video_yaw = roundf((current_video_yaw + 90.0f) / 360.0f) * 360.0f - 90.0f;
            }
            else
            {
                // R键正向状态：图传云台就近转回 0 度 (即正前方)
                robot->video_gimbal->Video_yaw = roundf(current_video_yaw / 360.0f) * 360.0f;
            }
        }
        last_shift_r = current_shift_r;

        // 旋转量
        float angle_buff = 0.15f;
        float angle_rapid_buff = 0.4f;
        if (robot->robot_mode == ROBOT_EXCHANGE_MODE)
        {
            if (chassis_ctrl_cmd->lift_ratio - 0.1f < 0.01f)
            {
                set_angle +=
                    (float)((rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].q - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].e) *
                                angle_buff +
                            (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].a - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].d) *
                                angle_rapid_buff);
            }
        }
        chassis_ctrl_cmd->robot_mode = robot->robot_mode;

        // ================= 4. 姿态复用控制 (Q, E, R) 与 兑换模式调节 =================
        if (robot->robot_mode == ROBOT_EXCHANGE_MODE)
        {
            // 【底盘高度】纯 QE 控制 (底盘逻辑，不用屏蔽)
            float step_size = 1.0f / (12.0f * 200.0f);
            chassis_ctrl_cmd->lift_ratio +=
                (float)(rc_data[TEMP].key[KEY_PRESS_NORMAL].q - rc_data[TEMP].key[KEY_PRESS_NORMAL].e) * step_size;

            if (chassis_ctrl_cmd->lift_ratio < 0.0f)
                chassis_ctrl_cmd->lift_ratio = 0.0f;
            if (chassis_ctrl_cmd->lift_ratio > 1.0f)
                chassis_ctrl_cmd->lift_ratio = 1.0f;

            // 【机械臂升降】Shift + W/S (不影响底盘)
            if (grab_control_mode == GRAB_CONTROL_KEYBOARD || grab_control_mode == GRAB_CONTROL_CUSTOM)
            {
                grab_ctrl_cmd->arm_lift +=
                    (float)(rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].w - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].s) *
                    grab_param.arm_lift_sens_keyboard;
            }

            if (grab_control_mode == GRAB_CONTROL_KEYBOARD)
            {
                // ================= 【机械臂 5 轴全控】 (Shift + Ctrl 组合键) =================
                float arm_speed = 0.08f; // 机械臂键盘微调步长

                // 1. 大 Yaw (基座旋转 base_joint) -> Q/W
                grab_ctrl_cmd->base_joint += (float)(rc_data[TEMP].key[KEY_PRESS_WITH_CTRL_SHIFT].q -
                                                     rc_data[TEMP].key[KEY_PRESS_WITH_CTRL_SHIFT].w) *
                                             arm_speed;

                // 2. 大 Roll (肘部旋转 elbow_roll) -> E/R
                grab_ctrl_cmd->elbow_roll += (float)(rc_data[TEMP].key[KEY_PRESS_WITH_CTRL_SHIFT].e -
                                                     rc_data[TEMP].key[KEY_PRESS_WITH_CTRL_SHIFT].r) *
                                             arm_speed;

                // 3. 大 Pitch (肘部俯仰 elbow_pitch) -> A/S
                grab_ctrl_cmd->elbow_pitch += (float)(rc_data[TEMP].key[KEY_PRESS_WITH_CTRL_SHIFT].a -
                                                      rc_data[TEMP].key[KEY_PRESS_WITH_CTRL_SHIFT].s) *
                                              arm_speed;

                // 4. 小 Pitch (腕部俯仰 wrist_pitch) -> D/F
                grab_ctrl_cmd->wrist_pitch += (float)(rc_data[TEMP].key[KEY_PRESS_WITH_CTRL_SHIFT].d -
                                                      rc_data[TEMP].key[KEY_PRESS_WITH_CTRL_SHIFT].f) *
                                              arm_speed;

                // 5. 小 Roll (腕部旋转 wrist_roll) -> Z/X
                grab_ctrl_cmd->wrist_roll += (float)(rc_data[TEMP].key[KEY_PRESS_WITH_CTRL_SHIFT].z -
                                                     rc_data[TEMP].key[KEY_PRESS_WITH_CTRL_SHIFT].x) *
                                             arm_speed;
            }
        }
    }

    // ================= 5. 夹爪控制 (Shift + Ctrl + C 触发式) =================
    uint32_t current_c_count = rc_data[TEMP].key_count[KEY_PRESS_WITH_CTRL_SHIFT][KEY_C];

    if (grab_control_mode == GRAB_CONTROL_KEYBOARD)
    {
        // 键鼠模式：根据按键奇偶次切换开关
        if (current_c_count % 2 == 1)
        {
            grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
        }
        else
        {
            grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
        }
    }
    else if (grab_control_mode == GRAB_CONTROL_CUSTOM)
    {
        // 自定义控制器模式：直接根据自制控制器传来的开/关状态赋值
        if (robot->self_control != NULL)
        {
            if (robot->self_control->unpacked_data.gripper_opened)
            {
                grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;
            }
            else
            {
                grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE;
            }
        }

        // 强制同步键鼠的 C 键次数，防止切回键鼠模式时发生冲突
        if (grab_ctrl_cmd->gripper_state == GRIPPER_CLOSE)
            rc_data[TEMP].key_count[KEY_PRESS_WITH_CTRL_SHIFT][KEY_C] = 1;
        else
            rc_data[TEMP].key_count[KEY_PRESS_WITH_CTRL_SHIFT][KEY_C] = 0;
    }
    else if (grab_control_mode == GRAB_CONTROL_HALF_AUTO)
    {
        // 半自动模式下同步键鼠状态
        if (grab_ctrl_cmd->gripper_state == GRIPPER_CLOSE)
            rc_data[TEMP].key_count[KEY_PRESS_WITH_CTRL_SHIFT][KEY_C] = 1;
        else
            rc_data[TEMP].key_count[KEY_PRESS_WITH_CTRL_SHIFT][KEY_C] = 0;
    }
    // ================= 6. 图传 Yaw/Pitch 控制 =================
    video_gimbal_ctrl_cmd->video_pitch =
        (float)(rc_data[TEMP].key[KEY_PRESS_NORMAL].x - rc_data[TEMP].key[KEY_PRESS_NORMAL].z) -
        (float)rc_data[TEMP].mouse.y * VIDEO_MOUSE_PITCH_SENS;

    video_gimbal_ctrl_cmd->video_yaw =
        (float)(rc_data[TEMP].key[KEY_PRESS_NORMAL].b - rc_data[TEMP].key[KEY_PRESS_NORMAL].v) +
        (float)rc_data[TEMP].mouse.x * VIDEO_MOUSE_YAW_SENS;

    // ================= 7. 机械臂与图传标定 =================
    if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].q)
    {
        grab_ctrl_cmd->wrist_roll_cali = 1;
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].e)
    {
        grab_ctrl_cmd->wrist_pitch_cali = 1;
    }
    // Ctrl+V：触发图传 Pitch 双向堵转标定
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].v)
    {
        video_gimbal_ctrl_cmd->video_cali = 1;
    }

    // ================= 8. UI 重置（Ctrl+B）=================
    // Ctrl+B（Back to default/Reset UI）：边沿触发，按下一次触发一次
    static uint8_t last_ctrl_b = 0;
    uint8_t curr_ctrl_b = rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].b;

    if (curr_ctrl_b && !last_ctrl_b) // 检测上升沿（按下瞬间）
    {
        robot->ui_reset_flag = 1; // 触发 UI 重置标志
    }

    last_ctrl_b = curr_ctrl_b; // 保存状态供下次比较
}

static void EmergencyHandler()
{
    if ((switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left)) ||
        !RemoteControlIsOnline())
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
    *rc_data_last = *rc_data;
    if (!robot->chassis->cali_state.all_cali_done)
    {
        if (RemoteControlIsOnline())
            chassis_ctrl_cmd->chassis_mode = CHASSIS_CALIBRATING;

        if (switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left))
            chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        return;
    }

    if (switch_is_mid(rc_data[TEMP].rc.switch_right))
    {
        if (abs(rc_data[TEMP].rc.dial) > 20)
        {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
            chassis_ctrl_cmd->wz = 0;
            set_angle += (rc_data[TEMP].rc.dial - 20) * 0.0001;
        }
        else
            chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    }
    else if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        if (chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_IDLE &&
            chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_BOTH_EXTEND &&
            chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_FRONT_RETRACT &&
            chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_ALL_RETRACT)
        {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_CLIMB_IDLE;
        }

        uint8_t current_switch = rc_data[TEMP].rc.switch_left;
        static uint8_t last_switch = 0;
        static uint32_t switch_stable_cnt = 0;

        if (current_switch == last_switch)
        {
            switch_stable_cnt++;
            if (switch_stable_cnt >= SWITCH_STABLE_TICKS)
            {
                if (switch_is_up(current_switch))
                    chassis_ctrl_cmd->chassis_mode = CHASSIS_CLIMB_BOTH_EXTEND;
                else if (switch_is_mid(current_switch))
                    chassis_ctrl_cmd->chassis_mode = CHASSIS_CLIMB_FRONT_RETRACT;
                else if (switch_is_down(current_switch))
                    chassis_ctrl_cmd->chassis_mode = CHASSIS_CLIMB_ALL_RETRACT;
                switch_stable_cnt = SWITCH_STABLE_TICKS;
            }
        }
        else
        {
            last_switch = current_switch;
            switch_stable_cnt = 0;
        }
    }

    // rocker_r1：向上推大于 300 夹紧，向下推小于 -300 松开
    if (rc_data[TEMP].rc.rocker_r1 > 300)
    {
        grab_ctrl_cmd->gripper_state = GRIPPER_CLOSE; // 👇 修改为状态
        if (rc_data[TEMP].key_count[KEY_PRESS_WITH_CTRL_SHIFT][KEY_C] % 2 == 0)
        {
            rc_data[TEMP].key_count[KEY_PRESS_WITH_CTRL_SHIFT][KEY_C]++;
        }
    }
    else if (rc_data[TEMP].rc.rocker_r1 < -300)
    {
        grab_ctrl_cmd->gripper_state = GRIPPER_OPEN;  // 👇 修改为状态
        if (rc_data[TEMP].key_count[KEY_PRESS_WITH_CTRL_SHIFT][KEY_C] % 2 == 1)
        {
            rc_data[TEMP].key_count[KEY_PRESS_WITH_CTRL_SHIFT][KEY_C]++;
        }
    }
    chassis_ctrl_cmd->vx = 60.0f * (float)rc_data[TEMP].rc.rocker_l_;
    chassis_ctrl_cmd->vy = 60.0f * (float)rc_data[TEMP].rc.rocker_l1;

    if (abs(rc_data[TEMP].rc.dial) > 20)
    {
        if (chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_ALL_RETRACT ||
            chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_FRONT_RETRACT)
        {
            set_angle += (rc_data[TEMP].rc.dial - 20) * 0.0001;
        }
        chassis_ctrl_cmd->wz = 0;
    }

    chassis_ctrl_cmd->wz = 0;
    *rc_data_last = *rc_data;
}

static void Gantry_Limit(Gantry_Ctrl_Cmd_s *gantry_ctrl_cmd, const Gantry_Param_s *gantry_param)
{
    static int32_t last_y, last_z;

    if (gantry_ctrl_cmd->z < 2200)
    {
        if (gantry_ctrl_cmd->y > 2200 && gantry_ctrl_cmd->y < 4000 && last_y < gantry_ctrl_cmd->y)
            gantry_ctrl_cmd->y = 2200;

        else if (gantry_ctrl_cmd->y < 11500 && gantry_ctrl_cmd->y > 9000 && last_y > gantry_ctrl_cmd->y)
            gantry_ctrl_cmd->y = 11500;

        if (gantry_ctrl_cmd->y > 2200 && gantry_ctrl_cmd->y < 11500 && last_z > gantry_ctrl_cmd->z)
            gantry_ctrl_cmd->z = 2100;
    }

    if (gantry_ctrl_cmd->z <= 0)
        gantry_ctrl_cmd->z = 0;
    else if (gantry_ctrl_cmd->z >= gantry_param->GANTRY_MAX_Z)
        gantry_ctrl_cmd->z = gantry_param->GANTRY_MAX_Z;

    last_z = gantry_ctrl_cmd->z;
}

static void ProcessCustomControllerData()
{
    if (robot->self_control != NULL)
    {
        if (robot->grab != NULL && grab_ctrl_cmd != NULL)
        {
            grab_ctrl_cmd->base_joint = SelfControlGetMotorAngle(robot->self_control, 3);
            grab_ctrl_cmd->elbow_pitch = SelfControlGetMotorAngle(robot->self_control, 1);
            grab_ctrl_cmd->wrist_pitch = SelfControlGetMotorAngle(robot->self_control, 2);
            grab_ctrl_cmd->wrist_roll = SelfControlGetMotorAngle(robot->self_control, 4);
            grab_ctrl_cmd->elbow_roll = SelfControlGetMotorAngle(robot->self_control, 0);
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

            custom_traj_length++;
            save_point_trigger = 0;
        }
    }
}