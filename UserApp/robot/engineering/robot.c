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
/* Intermediate variables calculated by private functions */
static RobotInstance *robot;
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;
static Grab_Ctrl_Cmd_s *grab_ctrl_cmd;
static Gantry_Ctrl_Cmd_s *gantry_ctrl_cmd; // 【新增】龙门架控制命令指针
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

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    robot->chassis = ChassisInit(&chassis_init_config);
#endif

    // 初始化控制命令指针
    chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
    chassis_ctrl_cmd->max_power = 80; // 随便给一个初始功率，后面应该要从裁判系统获取
    grab_ctrl_cmd = &robot->grab->grab_ctrl_cmd;
    // 【新增】龙门架控制命令指针
    // if (robot->gantry != NULL)
    // {
    //     gantry_ctrl_cmd = &robot->gantry->Gantry_ctrl_cmd;
    // }

    // gantry_param = gantry_init_config.Gantry_param;
    grab_param = grab_init_config.Grab_param;

    rc_data = robot->rc_data;
    gpio_5V_EN = GPIORegister(&gpio_init_config_5v);
    GPIOSet(gpio_5V_EN);
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
#if defined(ONE_BOARD) // 假设龙门架逻辑运行在主控板
    // GantryTask();
    GrabTask();
    // 机械臂使能由按键G控制，在MouseKeySet()中处理
    // grab_ctrl_cmd->grab_mode = b;
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
    // 只在自定义控制器模式下处理数据，避免与键鼠控制冲突
    Record_Current_Waypoint();
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
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
        return;
    }

    // ================= 1. 大模式切换 (按 G 键循环切换) =================
    // 状态顺序: 正常行车(0) -> 兑换(1) -> 上台阶(2)
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_G] % 3)
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
    }

    // 只有在非断电、非急停状态下，才允许执行后续控制指令
    // (断电和急停交由 EmergencyHandler 遥控器拨杆处理)
    if (robot->robot_mode != ROBOT_POWER_OFF && robot->robot_mode != ROBOT_EMERGENCY_STOP)
    {
        // 既然在工作模式，机械臂默认给电使能
        grab_ctrl_cmd->grab_mode = GRAB_POWER_ON;

        // ================= 2. 机械臂控制权切换 (仅在兑换模式下 F 键生效) =================
        if (robot->robot_mode == ROBOT_EXCHANGE_MODE)
        {
            switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 3)
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
            // 如果切出了兑换模式（比如去跑路或上台阶），强行把机械臂切回键鼠并锁定，防止外设误触
            grab_control_mode = GRAB_CONTROL_KEYBOARD;
            rc_data[TEMP].key_count[KEY_PRESS][Key_F] = 0;
        }

        // ================= 3. 底盘平移 (WASD 全局生效) =================
        float speed_buff = 20000; // 如果需要加速/减速，可以配合 Shift/Ctrl 修改此值
        chassis_ctrl_cmd->vx = (float)(rc_data[TEMP].key[KEY_PRESS].d - rc_data[TEMP].key[KEY_PRESS].a) * speed_buff;
        chassis_ctrl_cmd->vy = (float)(rc_data[TEMP].key[KEY_PRESS].w - rc_data[TEMP].key[KEY_PRESS].s) * speed_buff;
        float angle_buff = 0.0005f;
        if (robot->robot_mode == ROBOT_EXCHANGE_MODE)
        {
            if (chassis_ctrl_cmd->lift_ratio - 0.1f < 0.01f)
            {
                set_angle += (float)(rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].q - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].e) * angle_buff;
            }

        }
        else if (robot->robot_mode == ROBOT_CLIMB_MODE)
        {
            if (chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_ALL_RETRACT || chassis_ctrl_cmd->chassis_mode == CHASSIS_CLIMB_FRONT_RETRACT)
            {
                set_angle += (float)(rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].q - rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].e) * angle_buff;
            }
        }
        // 把大模式透传给底盘，否则底盘永远不知道切模式了
        chassis_ctrl_cmd->robot_mode = robot->robot_mode;

        // ================= 4. 姿态复用控制 (Q, E, R) 与 兑换模式无级调节 =================
        if (robot->robot_mode == ROBOT_CLIMB_MODE)
        {
            // 【上台阶模式】：控制四腿伸缩姿态
            if (rc_data[TEMP].key[KEY_PRESS].q)
            {
                chassis_ctrl_cmd->chassis_mode = CHASSIS_CLIMB_BOTH_EXTEND; // Q: 全升 (前伸后伸)
            }
            else if (rc_data[TEMP].key[KEY_PRESS].e)
            {
                chassis_ctrl_cmd->chassis_mode = CHASSIS_CLIMB_ALL_RETRACT; // E: 全收 (前收后收)
            }
            else if (rc_data[TEMP].key[KEY_PRESS].r)
            {
                chassis_ctrl_cmd->chassis_mode = CHASSIS_CLIMB_FRONT_RETRACT; // R: 半收 (前收后伸)
            }
        }
        else if (robot->robot_mode == ROBOT_EXCHANGE_MODE)
        {
            // 【兑换模式】底盘高度无级调节 (lift_ratio 0.0~1.0)
            // 目标：纯 QE 控制，15秒全伸展。任务频率 200Hz -> 总循环 3000 次 -> 步长 = 1.0f / 3000.0f
            float step_size = 1.0f / (15.0f * 200.0f); // 约等于 0.000333f

            // 可选：如果你希望在 15s 匀速的基础上，按 Shift 能加速/按 Ctrl 能龟速，解开下面两行注释
            // if (rc_data[TEMP].key[KEY_PRESS].shift) step_size *= 3.0f;  // Shift 极速模式 (5秒)
            // if (rc_data[TEMP].key[KEY_PRESS].ctrl)  step_size *= 0.2f;  // Ctrl 极慢微调模式 (75秒)

            // 只保留 Q/E 控制
            chassis_ctrl_cmd->lift_ratio +=
                (rc_data[TEMP].key[KEY_PRESS].q - rc_data[TEMP].key[KEY_PRESS].e) * step_size;

            // 安全限幅
            if (chassis_ctrl_cmd->lift_ratio < 0.0f)
                chassis_ctrl_cmd->lift_ratio = 0.0f;
            if (chassis_ctrl_cmd->lift_ratio > 1.0f)
                chassis_ctrl_cmd->lift_ratio = 1.0f;
        }
    }

    // ================= 5. 夹爪控制 (Shift + C 触发式) =================

    // 获取当前的计数值
    uint32_t current_c_count = rc_data[TEMP].key_count[KEY_PRESS_WITH_SHIFT][Key_C];
    static uint32_t last_grab_state = 0; // 0为松开状态，1为夹紧状态

    if (grab_control_mode == GRAB_CONTROL_KEYBOARD)
    {
        // 正常模式：根据计数值奇偶判断
        if (current_c_count % 2 == 1)
        {
            grab_ctrl_cmd->torque = 2.0f; // 夹紧
            last_grab_state = 1;
        }
        else
        {
            grab_ctrl_cmd->torque = -0.6f; // 松开
            last_grab_state = 0;
        }
    }
    else if (grab_control_mode == GRAB_CONTROL_CUSTOM)
    {
        current_selfcontrol_gripper = robot->self_control->unpacked_data.gripper_opened;
        if (current_selfcontrol_gripper != last_selfcontrol_gripper)
        {
            if (fabsf(grab_ctrl_cmd->torque + 0.6) < 0.01f)
            {
                grab_ctrl_cmd->torque = 2.0f;
            }
            else if (fabsf(grab_ctrl_cmd->torque - 2.0f) < 0.01f)
            {
                grab_ctrl_cmd->torque = -0.6f;
            }
        }
        else if (current_selfcontrol_gripper == last_selfcontrol_gripper)
        {
            grab_ctrl_cmd->torque = grab_ctrl_cmd->torque;
        }
        last_selfcontrol_gripper = current_selfcontrol_gripper;
        if (grab_ctrl_cmd->torque > 0.5f)
        {
            rc_data[TEMP].key_count[KEY_PRESS_WITH_SHIFT][Key_C] = 1;
        }
        else // 判定当前半自动是松开的
        {
            rc_data[TEMP].key_count[KEY_PRESS_WITH_SHIFT][Key_C] = 0;
        }
    }
    else if (grab_control_mode == GRAB_CONTROL_HALF_AUTO)
    {
        // 💥 核心修复：在半自动模式下，强行同步计数值
        // 如果半自动让夹爪关上了，我们就把键盘计数值强行同步成 1（奇数）
        // 如果半自动让夹爪开了，我们就把键盘计数值强行同步成 0（偶数）
        // 这样当你切回手动时，按键状态永远是和物理现状对齐的，不会乱跳！
        current_selfcontrol_gripper = robot->self_control->unpacked_data.gripper_opened;
        last_selfcontrol_gripper = current_selfcontrol_gripper;
        if (grab_ctrl_cmd->torque > 0.5f) // 判定当前半自动是夹紧的
        {
            rc_data[TEMP].key_count[KEY_PRESS_WITH_SHIFT][Key_C] = 1;
        }
        else // 判定当前半自动是松开的
        {
            rc_data[TEMP].key_count[KEY_PRESS_WITH_SHIFT][Key_C] = 0;
        }
    }
    // ================= 6. 图传云台控制 (ZX, VB) =================
    // Z/X: 控制普通图传 Pitch 俯仰角
    grab_ctrl_cmd->video_pitch +=
        (rc_data[TEMP].key[KEY_PRESS].z - rc_data[TEMP].key[KEY_PRESS].x) * grab_param.video_pitch_sens_keyboard;

    // V/B: 控制 3508 图传 Pitch (映射到原有的 video_forward 变量上)
    grab_ctrl_cmd->video_forward +=
        (rc_data[TEMP].key[KEY_PRESS].v - rc_data[TEMP].key[KEY_PRESS].b) * grab_param.video_forward_sens_keyboard;

    // ================= 机械臂标定 =================
    if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].q)
    {
        grab_ctrl_cmd->wrist_roll_cali = 1;
    }
    else if (rc_data[TEMP].key[KEY_PRESS_WITH_CTRL].e)
    {
        grab_ctrl_cmd->wrist_pitch_cali = 1;
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *.
 * @brief  紧急停止，人为手动急停（遥控器左右双拨杆打下）或者遥控器离线
 *
 */
static void EmergencyHandler()
{
    // 人为手动急停（遥控器左右双拨杆打下）或者遥控器离线
    if ((switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left)) ||
        !RemoteControlIsOnline())
    {
        robot->robot_mode = ROBOT_EMERGENCY_STOP;
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        // gantry_ctrl_cmd->Gantry_mode = GANTRY_MODE_POWER_OFF;
        grab_ctrl_cmd->grab_mode = GRAB_POWER_OFF;
        LOGINFO("[CMD] emergency stop!");
    }
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    *rc_data_last = *rc_data;
    if (!robot->chassis->cali_state.all_cali_done)
    {
        // 遥控器在线抢接管逻辑
        if (RemoteControlIsOnline())
        {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_CALIBRATING;
        }

        // 依然保留人为急停权限
        if (switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left))
        {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        }

        return;
    }

    // 右[中]：跟随模式
    if (switch_is_mid(rc_data[TEMP].rc.switch_right))
    {
        if (abs(rc_data[TEMP].rc.dial) > 20)
        {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
            chassis_ctrl_cmd->wz = 0;
            set_angle += (rc_data[TEMP].rc.dial - 20) * 0.0001;
        }
        else
        {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
        }
    }
    // 右[下]：控制底盘断电，但不触发整机紧急停止
    else if (switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    }
    // 右[上]：进入爬楼梯总模式，根据左拨杆细分姿态
    else if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        // 如果刚切过来，还没完成防抖，先给个 IDLE 状态，防止被拦截
        if (chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_IDLE &&
            chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_BOTH_EXTEND &&
            chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_FRONT_RETRACT &&
            chassis_ctrl_cmd->chassis_mode != CHASSIS_CLIMB_ALL_RETRACT)
        {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_CLIMB_IDLE;
        }

        // 获取当前左拨杆的瞬时状态进行防抖
        uint8_t current_switch = rc_data[TEMP].rc.switch_left;
        static uint8_t last_switch = 0;        // 上一次的拨杆位置
        static uint32_t switch_stable_cnt = 0; // 稳定计时器

        if (current_switch == last_switch)
        {
            switch_stable_cnt++;
            // 当达到 0.3s 的稳定时间时，执行状态切换
            if (switch_stable_cnt >= SWITCH_STABLE_TICKS)
            {
                if (switch_is_up(current_switch))
                {
                    chassis_ctrl_cmd->chassis_mode = CHASSIS_CLIMB_BOTH_EXTEND;
                }
                else if (switch_is_mid(current_switch))
                {
                    chassis_ctrl_cmd->chassis_mode = CHASSIS_CLIMB_FRONT_RETRACT;
                }
                else if (switch_is_down(current_switch))
                {
                    chassis_ctrl_cmd->chassis_mode = CHASSIS_CLIMB_ALL_RETRACT;
                }
                switch_stable_cnt = SWITCH_STABLE_TICKS;
            }
        }
        else
        {
            last_switch = current_switch;
            switch_stable_cnt = 0;
        }
    }
    // 底盘运动控制（使用左侧摇杆）
    chassis_ctrl_cmd->vx = 60.0f * (float)rc_data[TEMP].rc.rocker_l_; // 水平方向
    chassis_ctrl_cmd->vy = 60.0f * (float)rc_data[TEMP].rc.rocker_l1; // 竖直方向

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
/**
 * @brief 电控限位
 * @param gantry_ctrl_cmd 龙门架控制命令指针
 * @param gantry_param 龙门架参数指针
 */
static void Gantry_Limit(Gantry_Ctrl_Cmd_s *gantry_ctrl_cmd, const Gantry_Param_s *gantry_param)
{
    static int32_t last_x, last_y, last_z;

    if (gantry_ctrl_cmd->z < 2200)
    {
        if (gantry_ctrl_cmd->y > 2200 && gantry_ctrl_cmd->y < 4000 && last_y < gantry_ctrl_cmd->y)
            gantry_ctrl_cmd->y = 2200;

        else if (gantry_ctrl_cmd->y < 11500 && gantry_ctrl_cmd->y > 9000 && last_y > gantry_ctrl_cmd->y)
            gantry_ctrl_cmd->y = 11500;

        if (gantry_ctrl_cmd->y > 2200 && gantry_ctrl_cmd->y < 11500 && last_z > gantry_ctrl_cmd->z)
            gantry_ctrl_cmd->z = 2100;
    }

    // 抬升
    if (gantry_ctrl_cmd->z <= 0)
        gantry_ctrl_cmd->z = 0;
    else if (gantry_ctrl_cmd->z >= gantry_param->GANTRY_MAX_Z)
        gantry_ctrl_cmd->z = gantry_param->GANTRY_MAX_Z;

    // // 前伸
    // if (gantry_ctrl_cmd->y <= 0)
    //     gantry_ctrl_cmd->y = 0;
    // else if (gantry_ctrl_cmd->y >= gantry_param->GANTRY_MAX_Y)
    //     gantry_ctrl_cmd->y = gantry_param->GANTRY_MAX_Y;
    //
    // // 横移
    // if (gantry_ctrl_cmd->x <= 0)
    //     gantry_ctrl_cmd->x = 0;
    // else if (gantry_ctrl_cmd->x >= gantry_param->GANTRY_MAX_X)
    //     gantry_ctrl_cmd->x = gantry_param->GANTRY_MAX_X;

    // last_x = gantry_ctrl_cmd->x;
    // last_y = gantry_ctrl_cmd->y;
    last_z = gantry_ctrl_cmd->z;
}

/**
 * @brief 处理自定义控制器数据
 * @todo零点要写死，大pitch是反的
 */
static void ProcessCustomControllerData()
{
    if (robot->self_control != NULL)
    {
        // 直接获取电机和电位器角度数据
        if (robot->grab != NULL && grab_ctrl_cmd != NULL)
        {
            // 映射4个电机到机械臂关节 (根据实际硬件连接调整)
            grab_ctrl_cmd->base_joint = SelfControlGetMotorAngle(robot->self_control, 3);  // 电机3 -> 基座关节
            grab_ctrl_cmd->elbow_pitch = SelfControlGetMotorAngle(robot->self_control, 1); // 电机1 -> 肘部俯仰
            grab_ctrl_cmd->wrist_pitch = SelfControlGetMotorAngle(robot->self_control, 2); // 电机2 -> 腕部俯仰
            grab_ctrl_cmd->wrist_roll = SelfControlGetMotorAngle(robot->self_control, 4);  // 电机4 -> 腕部旋转
            grab_ctrl_cmd->elbow_roll = SelfControlGetMotorAngle(robot->self_control, 0);  // 电位器 -> 肘部旋转
        }
    }
}

static void CalcOffsetAngle()
{
    chassis_ctrl_cmd->offset_angle = set_angle - robot->ins_data->YawTotalAngle;
}

static void Record_Current_Waypoint(void)
{
    // 确保不会数组越界 (假设最大 50 步)

    if (save_point_trigger == 1)
    {
        if (custom_traj_length < 50)
        {
            // 记录机械臂的 6 个参数
            custom_trajectory[custom_traj_length][0] = grab_ctrl_cmd->base_joint;
            custom_trajectory[custom_traj_length][1] = grab_ctrl_cmd->elbow_roll;
            custom_trajectory[custom_traj_length][2] = grab_ctrl_cmd->elbow_pitch;
            custom_trajectory[custom_traj_length][3] = grab_ctrl_cmd->wrist_pitch;
            custom_trajectory[custom_traj_length][4] = grab_ctrl_cmd->wrist_roll;
            custom_trajectory[custom_traj_length][5] = grab_ctrl_cmd->torque;

            custom_trajectory[custom_traj_length][6] = chassis_ctrl_cmd->lift_ratio;

            custom_traj_length++; // 步数加 1
            save_point_trigger = 0;
        }
    }
}
/* ---------------------------------------------------------------------------*/