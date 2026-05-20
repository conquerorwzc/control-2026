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

static VideoGimbal_Ctrl_Cmd_s *video_gimbal_ctrl_cmd;
static VT13_RC_t *vt13_data;
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
    robot->vt13_data = VT13RemoteInit(&huart7); // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
#elifdef STM32H7
    // SelfControl 先注册，拥有 DMA buffer；VT13 作为 secondary callback 共享同一串口
    robot->self_control = SelfControlInit(&huart7);
    robot->vt13_data = VT13RemoteInitShared(robot->self_control->usart_instance);
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

    vt13_data = robot->vt13_data;
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
        if (vt13_data != NULL)
        {
            Half_auto_update(grab_ctrl_cmd, chassis_ctrl_cmd, vt13_data->mouse_key[TEMP].mouse.press_l, vt13_data->mouse_key[LAST].mouse.press_l,
                             vt13_data->mouse_key[TEMP].mouse.press_r, vt13_data->mouse_key[LAST].mouse.press_r);
        }
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
    if (vt13_data == NULL)
        return;

    if (abs(vt13_data->rc.dial) > 20 || vt13_data->rc.rocker_l1 != 0 || vt13_data->rc.rocker_l_ != 0 ||
        vt13_data->rc.rocker_r1 != 0 || vt13_data->rc.rocker_r_ != 0)
    {
        return; // 有摇杆输入时不进行键鼠控制
    }

    // 1. 把爬楼状态机变量提升到函数开头，方便全局复位
    static uint8_t keyboard_climb_state = CHASSIS_CLIMB_IDLE;
    // 记录上一次的大模式，用于边沿检测
    static uint8_t last_robot_mode = ROBOT_POWER_OFF;

    // ================= 1. 大模式切换 (按 G 键循环切换) =================
    switch (vt13_data->key_count.arr[KEY_PRESS_NORMAL][KEY_G] % 4)
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
        robot->robot_mode = ROBOT_DOWN_STAIRS_MODE;
        break;
    }

    if (robot->robot_mode != last_robot_mode)
    {
        // 🌟 新增：只要车体大模式发生了切换，直接强制清零一切半自动的残留步数！
        Half_auto_reset();

        // 场景：退出爬楼或烂路模式
        if (last_robot_mode == ROBOT_CLIMB_MODE || last_robot_mode == ROBOT_DOWN_STAIRS_MODE)
        {
            keyboard_climb_state = CHASSIS_CLIMB_IDLE;
        }

        // 场景：退出兑换模式
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

        // ================= 2. 机械臂控制权切换 (专属快捷键分离) =================
        if (robot->robot_mode == ROBOT_EXCHANGE_MODE || robot->robot_mode == ROBOT_CLIMB_MODE)
        {
            // 状态1：只按了 F 键 (此时 Ctrl 和 Shift 绝对没按)
            uint8_t curr_f_only = vt13_data->key.arr[KEY_PRESS_NORMAL].f;
            static uint8_t last_f_only = 0;

            // 状态2：按了 Ctrl + F (此时 Shift 绝对没按)
            uint8_t curr_ctrl_f = vt13_data->key.arr[KEY_PRESS_WITH_CTRL].f;
            static uint8_t last_ctrl_f = 0;

            // 场景 1：按下 Ctrl + F -> 无脑切入【半自动模式】
            if (curr_ctrl_f && !last_ctrl_f)
            {
                grab_control_mode = GRAB_CONTROL_HALF_AUTO;
            }

            // 场景 2：只按 F -> 在【键鼠】和【自定义遥控器】之间接管
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
            // 退出这两种大模式时，强制恢复为最安全的键鼠控制
            grab_control_mode = GRAB_CONTROL_KEYBOARD;
        }

        // ================= 3. 底盘平移 (WASD 全局生效) =================
        float speed_buff = 20000;

        // 提取 R 键的状态：0 为正向，1 为反向
        uint8_t r_state = vt13_data->key_count.arr[KEY_PRESS_NORMAL][KEY_R] % 2;

        if (r_state == 1)
        {
            // 反向平移
            chassis_ctrl_cmd->vx =
                -(float)(vt13_data->key.arr[KEY_PRESS_NORMAL].w - vt13_data->key.arr[KEY_PRESS_NORMAL].s) * speed_buff;
            chassis_ctrl_cmd->vy =
                (float)(vt13_data->key.arr[KEY_PRESS_NORMAL].d - vt13_data->key.arr[KEY_PRESS_NORMAL].a) * speed_buff;
        }
        else
        {
            // 正向平移
            chassis_ctrl_cmd->vx =
                (float)(vt13_data->key.arr[KEY_PRESS_NORMAL].d - vt13_data->key.arr[KEY_PRESS_NORMAL].a) * speed_buff;
            chassis_ctrl_cmd->vy =
                (float)(vt13_data->key.arr[KEY_PRESS_NORMAL].w - vt13_data->key.arr[KEY_PRESS_NORMAL].s) * speed_buff;
        }

        // ================= 新增：Shift+R 图传云台一键回正 =================
        static uint8_t last_shift_r = 0;
        uint8_t current_shift_r = vt13_data->key.arr[KEY_PRESS_WITH_SHIFT].r;

        // 边沿检测，按下的瞬间触发
        if (current_shift_r && !last_shift_r)
        {
            // 提取 R 键的状态：0 为正向，1 为侧向
             r_state = vt13_data->key_count.arr[KEY_PRESS_NORMAL][KEY_R] % 2;

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
                    (float)((vt13_data->key.arr[KEY_PRESS_WITH_SHIFT].q - vt13_data->key.arr[KEY_PRESS_WITH_SHIFT].e) *
                                angle_buff +
                            (vt13_data->key.arr[KEY_PRESS_WITH_SHIFT].a - vt13_data->key.arr[KEY_PRESS_WITH_SHIFT].d) *
                                angle_rapid_buff);
            }
        }
        else if (robot->robot_mode == ROBOT_CLIMB_MODE ||
                 robot->robot_mode == ROBOT_DOWN_STAIRS_MODE) // 🌟 修复2：向烂路模式开放键盘微调权限
        {
            // 🛡️ 物理防翻车护盾：绝对禁止在双腿全伸出的高重心状态下旋转！
            // 只有在全收、后腿半伸、后腿全伸等低重心状态下，才允许微调姿态
            if (chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_ALL_RETRACT ||
                chassis_ctrl_cmd->chassis_mode ==
                    CHASSIS_CLIMB_FRONT_RETRACT_REAR_HALF || // 🌟 修复1：加上后腿半伸的旋转权限
                chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_FRONT_RETRACT)
            {
                set_angle +=
                    (float)((vt13_data->key.arr[KEY_PRESS_WITH_SHIFT].q - vt13_data->key.arr[KEY_PRESS_WITH_SHIFT].e) *
                                angle_buff +
                            (vt13_data->key.arr[KEY_PRESS_WITH_SHIFT].a - vt13_data->key.arr[KEY_PRESS_WITH_SHIFT].d) *
                                angle_rapid_buff);
            }
        }
        chassis_ctrl_cmd->robot_mode = robot->robot_mode;

        // ================= 4. 姿态复用控制 (Q, E, R) 与 兑换模式调节 =================
        // 1. 底盘抬升逻辑 (根据大模式区分)
        if (robot->robot_mode == ROBOT_EXCHANGE_MODE)
        {
            // 【兑换模式】：使用平滑无级调节 (长按 QE)
            float step_size = 1.0f / (12.0f * 200.0f);
            chassis_ctrl_cmd->lift_ratio +=
                (float)(vt13_data->key.arr[KEY_PRESS_NORMAL].q - vt13_data->key.arr[KEY_PRESS_NORMAL].e) * step_size;

            if (chassis_ctrl_cmd->lift_ratio < 0.0f)
                chassis_ctrl_cmd->lift_ratio = 0.0f;
            if (chassis_ctrl_cmd->lift_ratio > 1.0f)
                chassis_ctrl_cmd->lift_ratio = 1.0f;
        }
   else if (robot->robot_mode == ROBOT_CLIMB_MODE || robot->robot_mode == ROBOT_DOWN_STAIRS_MODE)
        {
            // 【上台阶/烂路模式】：边沿触发状态机 (彻底告别组合键卡顿)
            static uint8_t last_ctrl_w = 0, last_q = 0, last_e = 0;

            uint8_t curr_ctrl_w = vt13_data->key.arr[KEY_PRESS_WITH_CTRL].w;
            uint8_t curr_q = vt13_data->key.arr[KEY_PRESS_NORMAL].q;
            uint8_t curr_e = vt13_data->key.arr[KEY_PRESS_NORMAL].e;

            // 1. Ctrl + W：四腿全伸 (准备上台阶 / 最大行程顶出标定)
            if (curr_ctrl_w && !last_ctrl_w)
            {
                keyboard_climb_state = CHASSIS_CLIMB_BOTH_EXTEND;
            }

            // 2. Q 键：前腿收，后腿伸 (爬台阶核心动作)
            if (curr_q && !last_q)
            {
                if (robot->robot_mode == ROBOT_DOWN_STAIRS_MODE)
                {
                    // 【下台阶模式】：两段式后伸逻辑 (半伸 -> 全伸)
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
                else
                {
                    // 【上台阶模式】：一段式逻辑 (直接全伸到位)
                    keyboard_climb_state = CHASSIS_CLIMB_FRONT_RETRACT;
                }
            }

            // 3. E 键：四腿全收 (上完台阶恢复底盘 / 物理原点)
            if (curr_e && !last_e)
            {
                keyboard_climb_state = CHASSIS_CLIMB_ALL_RETRACT;
            }

            last_ctrl_w = curr_ctrl_w;
            last_q = curr_q;
            last_e = curr_e;

            // 状态下发与护盾保护
            if (keyboard_climb_state != CHASSIS_CLIMB_IDLE)
            {
                // 只有在零点标定完全结束后，才允许执行爬楼姿态
                if (robot->chassis->cali_state.all_cali_done)
                {
                    chassis_ctrl_cmd->chassis_mode = keyboard_climb_state;
                }
            }
        }

        // 2. 机械臂全轴微调逻辑 (在兑换和上台阶下均可生效)
        if (robot->robot_mode == ROBOT_EXCHANGE_MODE || robot->robot_mode == ROBOT_CLIMB_MODE)
        {
            // 键鼠和自定义控制器情况下都能用
            if (grab_control_mode == GRAB_CONTROL_KEYBOARD || grab_control_mode == GRAB_CONTROL_CUSTOM)
            {
                // 1. 抬升：Shift + W/S
                grab_ctrl_cmd->arm_lift +=
                    (float)(vt13_data->key.arr[KEY_PRESS_WITH_SHIFT].w - vt13_data->key.arr[KEY_PRESS_WITH_SHIFT].s) *
                    grab_param.arm_lift_sens_keyboard;

                // 2. 前伸：Shift + Ctrl + V/B
                grab_ctrl_cmd->arm_extend += (float)(vt13_data->key.arr[KEY_PRESS_WITH_CTRL_SHIFT].v -
                                                     vt13_data->key.arr[KEY_PRESS_WITH_CTRL_SHIFT].b) *
                                             grab_param.arm_extend_sens_keyboard;
            }

            if (grab_control_mode == GRAB_CONTROL_KEYBOARD)
            {
                // ================= 【机械臂 5 轴全控】 (Shift + Ctrl 组合键) =================
                float arm_speed = 0.08f; // 机械臂键盘微调步长

                // 1. 大 Yaw (基座旋转 base_joint) -> Q/W
                grab_ctrl_cmd->base_joint += (float)(vt13_data->key.arr[KEY_PRESS_WITH_CTRL_SHIFT].q -
                                                     vt13_data->key.arr[KEY_PRESS_WITH_CTRL_SHIFT].w) *
                                             arm_speed;

                // 2. 大 Roll (肘部旋转 elbow_roll) -> E/R
                grab_ctrl_cmd->elbow_roll += (float)(vt13_data->key.arr[KEY_PRESS_WITH_CTRL_SHIFT].e -
                                                     vt13_data->key.arr[KEY_PRESS_WITH_CTRL_SHIFT].r) *
                                             arm_speed;

                // 3. 大 Pitch (肘部俯仰 elbow_pitch) -> A/S
                grab_ctrl_cmd->elbow_pitch += (float)(vt13_data->key.arr[KEY_PRESS_WITH_CTRL_SHIFT].a -
                                                      vt13_data->key.arr[KEY_PRESS_WITH_CTRL_SHIFT].s) *
                                              arm_speed;

                // 4. 小 Pitch (腕部俯仰 wrist_pitch) -> D/F
                grab_ctrl_cmd->wrist_pitch += (float)(vt13_data->key.arr[KEY_PRESS_WITH_CTRL_SHIFT].d -
                                                      vt13_data->key.arr[KEY_PRESS_WITH_CTRL_SHIFT].f) *
                                              arm_speed;

                // 5. 小 Roll (腕部旋转 wrist_roll) -> Z/X
                grab_ctrl_cmd->wrist_roll += (float)(vt13_data->key.arr[KEY_PRESS_WITH_CTRL_SHIFT].z -
                                                     vt13_data->key.arr[KEY_PRESS_WITH_CTRL_SHIFT].x) *
                                             arm_speed;
            }
        }
    }

    // ================= 5. 夹爪控制 (C 触发式) =================
    uint32_t current_c_count = vt13_data->key_count.arr[KEY_PRESS_NORMAL][KEY_C];

    if (grab_control_mode == GRAB_CONTROL_KEYBOARD || grab_control_mode == GRAB_CONTROL_CUSTOM)
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
    else if (grab_control_mode == GRAB_CONTROL_HALF_AUTO)
    {
        // 半自动模式下同步键鼠状态
        if (grab_ctrl_cmd->gripper_state == GRIPPER_CLOSE)
            vt13_data->key_count.arr[KEY_PRESS_NORMAL][KEY_C] = 1;
        else
            vt13_data->key_count.arr[KEY_PRESS_NORMAL][KEY_C] = 0;
    }
    // ================= 6. 图传 Yaw/Pitch 控制 =================
    video_gimbal_ctrl_cmd->video_pitch =
        (float)(vt13_data->key.arr[KEY_PRESS_NORMAL].x - vt13_data->key.arr[KEY_PRESS_NORMAL].z) -
        (float)vt13_data->mouse_key[TEMP].mouse.y * VIDEO_MOUSE_PITCH_SENS;

    video_gimbal_ctrl_cmd->video_yaw =
        (float)(vt13_data->key.arr[KEY_PRESS_NORMAL].b - vt13_data->key.arr[KEY_PRESS_NORMAL].v) +
        (float)vt13_data->mouse_key[TEMP].mouse.x * VIDEO_MOUSE_YAW_SENS;

    // ================= 7. 机械臂与图传标定 =================
    if (vt13_data->key.arr[KEY_PRESS_WITH_CTRL].q)
    {
        grab_ctrl_cmd->wrist_roll_cali = 1;
    }
    else if (vt13_data->key.arr[KEY_PRESS_WITH_CTRL].e)
    {
        grab_ctrl_cmd->wrist_pitch_cali = 1;
    }
    // Ctrl+V：触发图传 Pitch 双向堵转标定
    else if (vt13_data->key.arr[KEY_PRESS_WITH_CTRL].v)
    {
        video_gimbal_ctrl_cmd->video_cali = 1;
    }
    static uint8_t last_ctrl_z = 0;
    uint8_t curr_ctrl_z = vt13_data->key.arr[KEY_PRESS_WITH_CTRL].z;

    if (curr_ctrl_z && !last_ctrl_z)
    {
        grab_ctrl_cmd->arm_extend_cali = 1;
    }
    last_ctrl_z = curr_ctrl_z;
    // Ctrl+X：键盘拥有最高权限，强制重新触发底盘标定
    static uint8_t last_ctrl_x = 0;
    uint8_t curr_ctrl_x = vt13_data->key.arr[KEY_PRESS_WITH_CTRL].x;

    if (curr_ctrl_x && !last_ctrl_x) // 边沿触发，防止长按导致不断重置
    {
        // 主动剥夺标定完成标志位
        robot->chassis->cali_state.all_cali_done = 0;
        robot->chassis->cali_state.is_max_calibrated = 0;
        // 键盘直接下达标定指令，无视 has_calibrated_once 历史记录
        for (int i = 0; i < 4; i++)
        {
            robot->chassis->cali_state.max_cali_done[i] = 0;
        }
        chassis_ctrl_cmd->chassis_mode = CHASSIS_CALIBRATING;
    }
    last_ctrl_x = curr_ctrl_x;
    // ================= 8. UI 重置（Ctrl+B）=================
    // Ctrl+B（Back to default/Reset UI）：边沿触发，按下一次触发一次
    static uint8_t last_ctrl_b = 0;
    uint8_t curr_ctrl_b = vt13_data->key.arr[KEY_PRESS_WITH_CTRL].b;

    if (curr_ctrl_b && !last_ctrl_b) // 检测上升沿（按下瞬间）
    {
        robot->ui_reset_flag = 1; // 触发 UI 重置标志
    }

    last_ctrl_b = curr_ctrl_b; // 保存状态供下次比较
}

static void EmergencyHandler()
{
    // 状态机概念：非使能状态（左/中）或掉线 => 触发全局软件急停锁
    if (vt13_data->rc.mode_switch != 2 || !VT13RemoteIsOnline()) //
    {
        robot->robot_mode = ROBOT_EMERGENCY_STOP;          //
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF; //
        grab_ctrl_cmd->grab_mode = GRAB_POWER_OFF;          //
        video_gimbal_ctrl_cmd->power = VIDEO_POWER_OFF;    //
        LOGINFO("[SYS] Safety Shield: Emergency Stop Activated!"); //
    }
}

static void RemoteControlSet()
{
    if (vt13_data == NULL)
        return;

    // 进门第一步：永远清零 wz，杜绝无限累加（防疯转的核心）
    chassis_ctrl_cmd->wz = 0;

    // ================= 1. 未标定情况下的安全拦截 =================
    if (!robot->chassis->cali_state.all_cali_done)
    {
        // 非使能状态（左/中），底盘彻底断电瘫痪
        if (vt13_data->rc.mode_switch != 2)
        {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
            return;
        }

        // 使能状态下，如果是初次开机，强制去执行标定
        if (VT13RemoteIsOnline() && robot->chassis->cali_state.has_calibrated_once == 0)
        {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_CALIBRATING;
            return;
        }
        // 如果是中途掉线复活 (has_calibrated_once == 1)，不拦截，继续往下走接收控制
    }

    // 提取大模式状态
    bool is_keyboard_climb = (robot->robot_mode == ROBOT_CLIMB_MODE || robot->robot_mode == ROBOT_DOWN_STAIRS_MODE);

    // ================= 2. 全局使能状态（Switch == 右） =================
    if (vt13_data->rc.mode_switch == 2)
    {
        // 1. 底盘大模式切权：键盘处于爬台阶/过烂路模式时，不强制切回 FOLLOW
        if (!is_keyboard_climb)
        {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
        }

        // 2. 遥控器物理摇杆映射
        chassis_ctrl_cmd->vx = 60.0f * (float)vt13_data->rc.rocker_l_;
        chassis_ctrl_cmd->vy = 60.0f * (float)vt13_data->rc.rocker_l1;

        // 3. 拨轮微调（应用对称死区优化）
        int16_t dial_raw = vt13_data->rc.dial;
        float dial_processed = 0.0f;

        if (dial_raw > 20)       dial_processed = (float)(dial_raw - 20);
        else if (dial_raw < -20) dial_processed = (float)(dial_raw + 20);

        if (dial_processed != 0.0f)
        {
            // 物理防翻车护盾：如果不是爬楼模式，随便转；如果是爬楼模式，只有低重心姿态才允许转
            if (!is_keyboard_climb ||
                chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_ALL_RETRACT ||
                chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_FRONT_RETRACT_REAR_HALF ||
                chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_FRONT_RETRACT)
            {
                set_angle += dial_processed * 0.0001f;
            }
        }

        // 4. 【VT13 按钮控制】fn1, fn2, trigger
        static uint8_t last_fn1 = 0;
        static uint8_t last_fn2 = 0;
        static uint8_t last_trigger = 0;

        uint8_t curr_fn1 = vt13_data->rc.fn_1;
        uint8_t curr_fn2 = vt13_data->rc.fn_2;
        uint8_t curr_trigger = vt13_data->rc.trigger;

        // fn_1 rising edge: 标定最大尺寸
        if (curr_fn1 && !last_fn1)
        {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_CLIMB_BOTH_EXTEND;
            if (robot->grab != NULL && robot->grab->arm != NULL) {
                grab_ctrl_cmd->arm_extend = robot->grab->arm->max_extend;

            }
        }

        // fn_2 rising edge: 最小/全收
        if (curr_fn2 && !last_fn2)
        {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_CLIMB_ALL_RETRACT;
            grab_ctrl_cmd->arm_extend = 0.0f;
        }

        // trigger rising edge: 夹抓开/关
        if (curr_trigger && !last_trigger)
        {
            grab_ctrl_cmd->gripper_state =
                (grab_ctrl_cmd->gripper_state == GRIPPER_CLOSE) ? GRIPPER_OPEN : GRIPPER_CLOSE;
        }

        last_fn1 = curr_fn1;
        last_fn2 = curr_fn2;
        last_trigger = curr_trigger;
    }
    else
    // ================= 3. 非使能状态（Switch == 左/中） =================
    {
        // 彻底切断物理遥控器的输出指令
        chassis_ctrl_cmd->vx = 0.0f;
        chassis_ctrl_cmd->vy = 0.0f;
    }
}

static void ProcessCustomControllerData()
{
    if (robot->self_control != NULL)
    {
        if (robot->grab != NULL && grab_ctrl_cmd != NULL)
        {
            // motor[0] = 大yaw (M6020, ID:1)
            grab_ctrl_cmd->base_joint = SelfControlGetMotorAngle(robot->self_control, 0);

            // motor[1] = 大roll (DM4340, ID:0x04, MasterID:0x14)
            grab_ctrl_cmd->elbow_roll = SelfControlGetMotorAngle(robot->self_control, 1);

            // motor[2] = 大pitch (DM4310, ID:0x03, MasterID:0x13)
            grab_ctrl_cmd->elbow_pitch = SelfControlGetMotorAngle(robot->self_control, 2);

            // motor[3] = 小pitch (DM4310, ID:0x02, MasterID:0x12)
            grab_ctrl_cmd->wrist_pitch = SelfControlGetMotorAngle(robot->self_control, 3);

            // motor[4] = 小roll (DM4310, ID:0x01, MasterID:0x11)
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
            // [0]-[4]：机械臂 5 个旋转关节
            custom_trajectory[custom_traj_length][0] = grab_ctrl_cmd->base_joint;
            custom_trajectory[custom_traj_length][1] = grab_ctrl_cmd->elbow_roll;
            custom_trajectory[custom_traj_length][2] = grab_ctrl_cmd->elbow_pitch;
            custom_trajectory[custom_traj_length][3] = grab_ctrl_cmd->wrist_pitch;
            custom_trajectory[custom_traj_length][4] = grab_ctrl_cmd->wrist_roll;

            // [5]：夹爪状态 (1.0 为闭合，0.0 为开启)
            custom_trajectory[custom_traj_length][5] = (grab_ctrl_cmd->gripper_state == GRIPPER_CLOSE) ? 1.0f : 0.0f;

            // [6]：底盘四腿抬升比例 (兑换模式使用)
            custom_trajectory[custom_traj_length][6] = chassis_ctrl_cmd->lift_ratio;

            // [7]：机械臂前伸推杆 (直线行程)
            custom_trajectory[custom_traj_length][7] = grab_ctrl_cmd->arm_extend;

            // [8]：机械臂整体抬升机构 (上下行程)
            custom_trajectory[custom_traj_length][8] = grab_ctrl_cmd->arm_lift;

            custom_traj_length++;
            save_point_trigger = 0;
        }
    }
}

/**
 * @brief 定时发送机械臂电机数据给自定义控制器 (10Hz)
 * @note RobotTask运行在200Hz,此处通过时间戳控制为10Hz
 *       复用裁判系统的USART1实例
 */
static void SendArmMotorDataTask(void)
{
    static uint32_t last_send_time = 0;

    // 10Hz频率控制: 距离上次发送不足100ms则跳过
    uint32_t current_time = HAL_GetTick();
    if ((current_time - last_send_time) < 100)
    {
        return;
    }

    // 获取机械臂5个关节角度
    float motor_angles[5] = {0.0f};
    if (robot->grab != NULL && robot->grab->arm != NULL && robot->grab->actuator != NULL)
    {
        // 根据控制模式选择数据源
        if (grab_control_mode == GRAB_CONTROL_HALF_AUTO)
        {
            // 半自动模式：发送指令角度（目标位置）
            motor_angles[0] = robot->grab->grab_ctrl_cmd.base_joint;  // 基座关节
            motor_angles[1] = robot->grab->grab_ctrl_cmd.elbow_roll;  // 肘部滚转
            motor_angles[2] = robot->grab->grab_ctrl_cmd.elbow_pitch; // 肘部俯仰
            motor_angles[3] = robot->grab->grab_ctrl_cmd.wrist_pitch; // 腕部俯仰
            motor_angles[4] = robot->grab->grab_ctrl_cmd.wrist_roll;  // 腕部滚转
        }
        else
        {
            // 自定义控制器模式：发送实时测量角度（实际位置）
            motor_angles[0] = robot->grab->grab_measure.base_joint;  // 基座关节
            motor_angles[1] = robot->grab->grab_measure.elbow_roll;  // 肘部滚转
            motor_angles[2] = robot->grab->grab_measure.elbow_pitch; // 肘部俯仰
            motor_angles[3] = robot->grab->grab_measure.wrist_pitch; // 腕部俯仰
            motor_angles[4] = robot->grab->grab_measure.wrist_roll;  // 腕部滚转
        }
    }

    // 获取当前机械臂控制模式并发送
    uint8_t control_mode = (uint8_t)grab_control_mode;

    // 使用selfcontrol的USART实例发送（UART7，921600波特率）
    if (robot->self_control != NULL && robot->self_control->usart_instance != NULL)
    {
        SelfControl_SendMotorDataToCustom(motor_angles, control_mode, robot->self_control->usart_instance);
    }

    // 更新发送时间戳
    last_send_time = current_time;
}
