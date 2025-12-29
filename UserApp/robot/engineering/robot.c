/* Private includes ----------------------------------------------------------*/
#include "robot.h"
#include "cmsis_os.h"
#include "general_def.h"
#include "ins_task.h"
#include "robot_config.h"
#include "stdlib.h"
#include "string.h"
#include "user_lib.h"
#include "semi_automatic.h" // 添加半自动控制头文件

/* Private define ------------------------------------------------------------*/

/* Intermediate variables calculated by private functions */
static RobotInstance *robot;
static Chassis_Ctrl_Cmd_s *chassis_ctrl_cmd;
static Grab_Ctrl_Cmd_s *grab_ctrl_cmd;
static Gantry_Ctrl_Cmd_s *gantry_ctrl_cmd; // 【新增】龙门架控制命令指针
static SemiAuto_Ctrl_Cmd_s *semi_auto_ctrl_cmd; // 半自动控制命令指针
static RC_ctrl_t *rc_data;
static RC_ctrl_t *rc_data_last; // 遥控器数据,初始化时返回
static float set_angle = 0;
static void MouseKeySet();

int b = 0;
static float angle = 0;
static float target_angle = 0;
static int mouse_l_count = 0;
static Gantry_Param_s gantry_param;
static Garb_Param_s grab_param;
static SemiAuto_Param_s semi_auto_param; // 半自动参数
/* Private function prototypes -----------------------------------------------*/
static void Gantry_Limit(Gantry_Ctrl_Cmd_s *gantry_ctrl_cmd, const Gantry_Param_s *gantry_param);
static void Grab_Limit(Grab_Ctrl_Cmd_s *grab_ctrl_cmd, const Gantry_Param_s *gantry_param);
static void RemoteControlSet();

static void EmergencyHandler();

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
#endif

    rc_data_last = (RC_ctrl_t *)zmalloc(sizeof(RC_ctrl_t));
    *rc_data_last = *robot->rc_data; // 记录上一次遥控器的状态
    robot->ins_data = INS_Init(&imu_init_config);

#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    robot->chassis = ChassisInit(&chassis_init_config);
#endif
    robot->gantry = GantryInit(&gantry_init_config);
    robot->grab = GrabInit(&grab_init_config);

    // 初始化半自动控制参数
    semi_auto_param.gantry_lift_pos = 3000.0f;      // 龙门架抬升目标位置
    semi_auto_param.chassis_forward_speed = 40000.0f; // 底盘前移速度（用于插入操作）
    semi_auto_param.arm_raise_angle = 10.0f;        // 机械臂上抬角度
    semi_auto_param.handle_flip_angle = 15.0f;      // 把手掰动角度
    semi_auto_param.rotate_angle = 5.0f;            // 旋转角度（5度）
    semi_auto_param.step_delay_ms = 1000;           // 步骤间延时（毫秒）

    // 初始化半自动控制模块
    SemiAuto_Init_Config_s semi_auto_init_config;
    semi_auto_init_config.param = semi_auto_param;
    robot->semi_auto = SemiAutoInit(&semi_auto_init_config);
    
    // 将龙门架、机械臂和底盘实例赋给半自动控制模块
    if (robot->semi_auto != NULL) {
        robot->semi_auto->gantry = robot->gantry;
        robot->semi_auto->grab = robot->grab;
        robot->semi_auto->chassis = robot->chassis;
    }

    // 初始化控制命令指针
    chassis_ctrl_cmd = &robot->chassis->chassis_ctrl_cmd;
    chassis_ctrl_cmd->max_power = 80; // 随便给一个初始功率，后面应该要从裁判系统获取
    grab_ctrl_cmd = &robot->grab->grab_ctrl_cmd;
    // 【新增】龙门架控制命令指针
    if (robot->gantry != NULL)
    {
        gantry_ctrl_cmd = &robot->gantry->Gantry_ctrl_cmd;
    }
    
    // 【新增】半自动控制命令指针
    if (robot->semi_auto != NULL)
    {
        semi_auto_ctrl_cmd = &robot->semi_auto->ctrl_cmd;
    }

    gantry_param = gantry_init_config.Gantry_param;
    grab_param = grab_init_config.Grab_param;

    rc_data = robot->rc_data;
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
    GantryTask();
    GrabTask();
    SemiAutoTask(); // 添加半自动控制任务
    // grab_ctrl_cmd->grab_mode = b;
#endif
}

/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    CalcOffsetAngle();
    RemoteControlSet();
    MouseKeySet();
    EmergencyHandler(); // 处理模块离线和遥控器急停等紧急情况
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    if (rc_data == NULL)
    {
        return;
    }

    if (rc_data[TEMP].rc.dial != 0 || rc_data[TEMP].rc.rocker_l1 != 0 || rc_data[TEMP].rc.rocker_l_ != 0 ||
        rc_data[TEMP].rc.rocker_r1 != 0 || rc_data[TEMP].rc.rocker_r_ != 0)
    {
        return; // 有摇杆输入时不进行键鼠控制
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_G] % 2) // G键控制机械臂使能
    {
    case 0:
        grab_ctrl_cmd->grab_mode = GRAB_POWER_OFF;
        break;
    case 1:
        grab_ctrl_cmd->grab_mode = GRAB_POWER_ON;
        break;
    default:
        break;
    }
    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 4) // C键设置底盘速度
    {
    case 0:
        chassis_ctrl_cmd->chassis_speed_buff = 10000;
        break;
    case 1:
        chassis_ctrl_cmd->chassis_speed_buff = 20000;
        break;
    case 2:
        chassis_ctrl_cmd->chassis_speed_buff = 40000;
        break;
    default:
        chassis_ctrl_cmd->chassis_speed_buff = 80000;
        break;
    }

    // 添加独立抬升龙门架控制 - 使用F键抬升龙门架
    if (rc_data[TEMP].key[KEY_PRESS].f) {
        if (robot->semi_auto != NULL) {
            LiftGantryToTarget(); // 独立抬升龙门架
        }
    }

    // 添加半自动操作控制 - 使用H键启动半自动操作（从插入矿物开始）
    if (rc_data[TEMP].key_count[KEY_PRESS][Key_H] % 2 == 1) {
        if (semi_auto_ctrl_cmd != NULL && !semi_auto_ctrl_cmd->is_running) {
            StartSemiAutoOperation(); // 启动半自动操作（从插入矿物开始）
        }
    }

    // 使用J键停止半自动操作
    if (rc_data[TEMP].key[KEY_PRESS].j) {
        if (semi_auto_ctrl_cmd != NULL) {
            StopSemiAutoOperation(); // 停止半自动操作
        }
    }

    if (gantry_ctrl_cmd->Gantry_mode != GANTRY_MODE_POWER_OFF)
    {
        gantry_ctrl_cmd->y += rc_data[TEMP].key[KEY_PRESS].b * 0.1 - rc_data[TEMP].key[KEY_PRESS].v * 0.1;
        gantry_ctrl_cmd->z += rc_data[TEMP].key[KEY_PRESS].x * 0.1 - rc_data[TEMP].key[KEY_PRESS].z * 0.1;
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 3) // 控制底盘/机械臂/摄像头
    {
    case 0: // 控制底盘
        if (chassis_ctrl_cmd->chassis_mode != CHASSIS_POWER_OFF)
        {
            chassis_ctrl_cmd->vx = rc_data[TEMP].key[KEY_PRESS].d * chassis_ctrl_cmd->chassis_speed_buff -
                                   rc_data[TEMP].key[KEY_PRESS].a * chassis_ctrl_cmd->chassis_speed_buff;
            chassis_ctrl_cmd->vy = rc_data[TEMP].key[KEY_PRESS].w * chassis_ctrl_cmd->chassis_speed_buff -
                                   rc_data[TEMP].key[KEY_PRESS].s * chassis_ctrl_cmd->chassis_speed_buff;
            // set_angle += -rc_data[TEMP].mouse.x * 0.001;
        }
        break;

    case 1: // 控制机械臂

        // 用adwsqe控制机械臂
        if (rc_data[TEMP].key[KEY_PRESS].keys != 0 && rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].keys == 0 &&
            grab_ctrl_cmd->grab_mode == GRAB_POWER_ON)
        {
            grab_ctrl_cmd->base_joint +=
                (rc_data[TEMP].key[KEY_PRESS].a - rc_data[TEMP].key[KEY_PRESS].d) * grab_param.base_joint_sens_keyboard;
            grab_ctrl_cmd->elbow_pitch += (rc_data[TEMP].key[KEY_PRESS].w - rc_data[TEMP].key[KEY_PRESS].s) *
                                          grab_param.elbow_pitch_sens_keyboard;
            grab_ctrl_cmd->elbow_roll +=
                (rc_data[TEMP].key[KEY_PRESS].q - rc_data[TEMP].key[KEY_PRESS].e) * grab_param.elbow_roll_sens_keyboard;
        }
        // 用shift+wasd控制腕部
        else if (rc_data[TEMP].key[KEY_PRESS].keys != 0 && rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].keys != 0 &&
                 grab_ctrl_cmd->grab_mode == GRAB_POWER_ON)
        {
            grab_ctrl_cmd->wrist_roll += (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].d - rc_data[TEMP].key[KEY_PRESS].a) *
                                         grab_param.wrist_roll_sens_keyboard;
            grab_ctrl_cmd->wrist_pitch += (rc_data[TEMP].key[KEY_PRESS_WITH_SHIFT].w - rc_data[TEMP].key[KEY_PRESS].s) *
                                          grab_param.wrist_pitch_sens_keyboard;
            switch (rc_data[TEMP].key_count[KEY_PRESS_WITH_SHIFT][Key_C] % 2)
            {
            case 0:
                grab_ctrl_cmd->torque = -0.6;
                break;
            case 1:
                grab_ctrl_cmd->torque = 2;
                break;
            default:
                break;
            }
        }
        break;

    case 2: // 控制摄像头

        grab_ctrl_cmd->vedio_forward +=
            (rc_data[TEMP].key[KEY_PRESS].d - rc_data[TEMP].key[KEY_PRESS].a) * grab_param.vedio_forward_sens_keyboard;

        grab_ctrl_cmd->vedio_pitch +=
            (rc_data[TEMP].key[KEY_PRESS].w - rc_data[TEMP].key[KEY_PRESS].s) * grab_param.vedio_pitch_sens_keyboard;

        break;
    default:
        break;
    }
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *.
 */
static void EmergencyHandler()
{
    // 遥控器不在线的时候停止所有电机
    if ((switch_is_down(rc_data[TEMP].rc.switch_right) && switch_is_down(rc_data[TEMP].rc.switch_left)) ||
        !RemoteControlIsOnline())
    {
        robot->robot_mode = ROBOT_EMERGENCY_STOP;
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
        gantry_ctrl_cmd->Gantry_mode = GANTRY_MODE_POWER_OFF;
        grab_ctrl_cmd->grab_mode = GRAB_POWER_OFF;
        // 紧急停止时也停止半自动操作
        if (semi_auto_ctrl_cmd != NULL) {
            StopSemiAutoOperation();
        }
        LOGINFO("[CMD] emergency stop!");
    }
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置wasdqezxcvbg
 *
 */
static void RemoteControlSet()
{
    // 右侧拨杆控制底盘模式
    if (switch_is_mid(rc_data[TEMP].rc.switch_right))
    {
        if (abs(rc_data[TEMP].rc.dial) > 20)
        {
            chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
            set_angle += rc_data[TEMP].rc.dial * 0.0001;
        }
        else
            chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
    }
    // 右[上]，保持底盘跟随云台
    else if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {

        chassis_ctrl_cmd->chassis_mode = CHASSIS_FOLLOW;
    }
    // 右[下] 控制底盘断电，但不触发整机紧急停止
    else if (switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        chassis_ctrl_cmd->chassis_mode = CHASSIS_POWER_OFF;
    }

    // 左侧拨杆控制龙门架模式
    if (gantry_ctrl_cmd != NULL)
    {
        if (switch_is_up(rc_data[TEMP].rc.switch_left))
        {
            // 左[上]：遥控器控制龙门架
            gantry_ctrl_cmd->Gantry_mode = GANTRY_MODE_CONTROL_REMOTE;
        }
        else if (switch_is_mid(rc_data[TEMP].rc.switch_left))
        {
            // 左[中]：龙门架锁死
            gantry_ctrl_cmd->Gantry_mode = GANTRY_MODE_LOCK;
        }
        else if (switch_is_down(rc_data[TEMP].rc.switch_left))
        {
            // 左[下]：龙门架断电
            gantry_ctrl_cmd->Gantry_mode = GANTRY_MODE_POWER_OFF;
        }

        // 遥控模式控制龙门架
        if (gantry_ctrl_cmd->Gantry_mode == GANTRY_MODE_CONTROL_REMOTE)
        {
            if (rc_data != NULL)
            {
                gantry_ctrl_cmd->x += rc_data[TEMP].rc.rocker_r_ * gantry_init_config.Gantry_param.sidesway_sens_remote;
                gantry_ctrl_cmd->y += rc_data[TEMP].rc.rocker_r1 * gantry_init_config.Gantry_param.stretch_sens_remote;
                gantry_ctrl_cmd->z += rc_data[TEMP].rc.rocker_l1 * gantry_init_config.Gantry_param.lift_sens_remote;
            }
        }

        // 执行龙门架限位（仅在非断电模式下）
        if (gantry_ctrl_cmd->Gantry_mode != GANTRY_MODE_POWER_OFF)
        {
            Gantry_Limit(gantry_ctrl_cmd, &robot->gantry->Gantry_param);
        }
    }

    // 底盘运动控制（使用左侧摇杆）
    chassis_ctrl_cmd->vx = 60.0f * (float)rc_data[TEMP].rc.rocker_l1; // 水平方向
    chassis_ctrl_cmd->vy = 60.0f * (float)rc_data[TEMP].rc.rocker_l_; // 竖直方向

    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_FOLLOW)
    {
        set_angle += rc_data[TEMP].rc.dial * 0.0001;
    }
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

    // 前伸
    if (gantry_ctrl_cmd->y <= 0)
        gantry_ctrl_cmd->y = 0;
    else if (gantry_ctrl_cmd->y >= gantry_param->GANTRY_MAX_Y)
        gantry_ctrl_cmd->y = gantry_param->GANTRY_MAX_Y;

    // 横移
    if (gantry_ctrl_cmd->x <= 0)
        gantry_ctrl_cmd->x = 0;
    else if (gantry_ctrl_cmd->x >= gantry_param->GANTRY_MAX_X)
        gantry_ctrl_cmd->x = gantry_param->GANTRY_MAX_X;

    last_x = gantry_ctrl_cmd->x;
    last_z = gantry_ctrl_cmd->z;
    last_y = gantry_ctrl_cmd->y;
}

static void CalcOffsetAngle()
{
    chassis_ctrl_cmd->offset_angle = set_angle - robot->ins_data->YawTotalAngle;
}
/* ---------------------------------------------------------------------------*/