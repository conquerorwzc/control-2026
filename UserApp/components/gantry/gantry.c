#include "gantry.h"
#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "remote_control.h"
#include "math.h"
#include "stdlib.h"


// #define GANTRY_LIFT_SENS_REMOTE 0.015     // 抬升电机灵敏度(遥控器)
// #define GANTRY_STRETCH_SENS_REMOTE 0.013  // 前伸电机灵敏度(遥控器)
// #define GANTRY_SIDESWAY_SENS_REMOTE 0.015 // 横移电机灵敏度(遥控器)
//
// #define GANTRY_LIFT_SENS_KEYBOARD 7     // 抬升电机灵敏度(键鼠)
// #define GANTRY_STRETCH_SENS_KEYBOARD 6  // 前伸电机灵敏度(键鼠)
// #define GANTRY_SIDESWAY_SENS_KEYBOARD 7 // 横移电机灵敏度(键鼠)
//
// #define GANTRY_MAX_Y 19000 // 前伸最前位置
// #define GANTRY_MAX_Z 40000 // 抬升最高位置
// #define GANTRY_MAX_X 17000 // 横移最右位置
//
// #define POSITION_ECD_RATIO 30 // 位置矢量与电机转动角度的比例

// 使用新的实例结构体
static GantryInstance* gantry_instance = NULL;

// 初始化位置
static uint16_t init_lift_ecd[2], init_stretch_ecd[2], init_sidesway_ecd;// 电机刚上电时的位置
static float last_gantry_x;
static uint8_t sidesway_st = 0; // 用于上电初始化时告诉机械臂横移已经移动到位

extern RC_ctrl_t controller;     // 自定义控制器数据
extern DJI_Motor_Measure_s camera_motor_lift; // 相机平台的抬升电机

static void Gantry_Run(GantryInstance *gantry);
static void Gantry_Init_Motors(GantryInstance *gantry);
static void Gantry_Control(GantryInstance *gantry);
static void Gantry_Limit(GantryInstance *gantry);
static void Gantry_Position_Calculate(GantryInstance *gantry);
static void Gantry_Pid_Calculate(GantryInstance *gantry);
static void Gantry_Can_Cmd(GantryInstance *gantry);
static void Gantry_Controller_Control(GantryInstance *gantry);
static void Gantry_Keyboard_Control(GantryInstance *gantry);
static void Gantry_Remote_Control(GantryInstance *gantry);

/**
 * @brief 龙门架任务函数
 */
void StartGantryTask(void const *argument)
{
    // 等待实例初始化完成
    while (gantry_instance == NULL) {
        osDelay(10);
    }

    uint32_t wait_time = xTaskGetTickCount();
    for (;;)
    {
        Gantry_Run(gantry_instance);
        osDelayUntil(&wait_time, 1);
    }
}

static void Gantry_Run(GantryInstance *gantry)
{
    Gantry_Control(gantry);            // 根据相应控制模式控制龙门架
    Gantry_Limit(gantry);              // 电控限位
    Gantry_Position_Calculate(gantry); // 根据位置矢量解算出电机转动的圈数
    Gantry_Pid_Calculate(gantry);      // 龙门架PID计算
    Gantry_Can_Cmd(gantry);            // 使用pid计算结果控制电机转动
    // 【新增】调用电机模块的发送函数，将电流值发送给电机
    DJIMotorTask();
}

/**
 * @brief 初始化龙门架电机，将龙门架移动到初始位置
 * @param gantry 龙门架实例指针
 */
static void Gantry_Init_Motors(GantryInstance *gantry)
{
    sidesway_st = 1; // 告诉机械臂横移已经移动到位
}

/**
 * @brief 龙门架初始化
 * @param init_config 初始化配置
 * @return GantryInstance* 龙门架实例指针
 */
GantryInstance* GantryInit(Gantry_Init_Config_s* init_config)
{
    if (gantry_instance != NULL) {
        return gantry_instance; // 已经初始化过
    }

    gantry_instance = (GantryInstance*)malloc(sizeof(GantryInstance));
    if (gantry_instance == NULL) {
        return NULL;
    }
    memset(gantry_instance, 0, sizeof(GantryInstance));

    // 保存参数配置
    gantry_instance->Gantry_param = init_config->Gantry_param;
    // 初始化控制命令
    gantry_instance->Gantry_ctrl_cmd.Gantry_mode = GANTRY_MODE_POWER_OFF;

    // 初始化电机
    for (int i = 0; i < 2; i++)
    {
        // 抬升电机
        gantry_instance->lift_motor[i].motor = DJIMotorInit(&init_config->lift_motor_config[i]);
        // 前伸电机
        gantry_instance->stretch_motor[i].motor = DJIMotorInit(&init_config->stretch_motor_config[i]);
    }
    // 横移电机
    gantry_instance->sidesway_motor.motor = DJIMotorInit(&init_config->sidesway_motor_config);

    // 设置遥控器数据指针（需要在外部初始化）
    // gantry_instance->remote_data = ...;
    // gantry_instance->keyboard = ...;

    // 等电机数据出现代表电机已经正常工作
    for (int i = 0; i < 2; i++)
    {
        while (gantry_instance->lift_motor[i].motor == NULL ||
               gantry_instance->lift_motor[i].motor->measure.ecd == 0)
            osDelay(10);
        while (gantry_instance->stretch_motor[i].motor == NULL ||
               gantry_instance->stretch_motor[i].motor->measure.ecd == 0)
            osDelay(10);
    }
    while (gantry_instance->sidesway_motor.motor == NULL ||
           gantry_instance->sidesway_motor.motor->measure.ecd == 0)
        osDelay(10);

    osDelay(100);

    // 设置目标角度为当前角度，防止上电时失控
    for (int i = 0; i < 2; i++)
    {
        // 注意：这里需要根据实际电机实例进行调整
        init_lift_ecd[i] = gantry_instance->lift_motor[i].motor->measure.ecd;
        init_stretch_ecd[i] = gantry_instance->stretch_motor[i].motor->measure.ecd;
    }
    init_sidesway_ecd = gantry_instance->sidesway_motor.motor->measure.ecd;

    // 切换到遥控器控制模式
    gantry_instance->Gantry_ctrl_cmd.Gantry_mode = GANTRY_MODE_CONTROL_REMOTE;

    // Gantry_Motor_Init(gantry);
    return gantry_instance;
}

/**
 * @brief 自定义控制器限制跟随速度
 * @param dir 当前位置
 * @param target 目标位置
 * @param sens 灵敏度
 */
static void Gantry_Controller_Limit_Speed(float *dir, float target, float sens)
{
    if (*dir - sens > target) // 目标位置比当前小
        *dir -= sens;
    else if (*dir + sens < target) // 目标位置比当前大
        *dir += sens;
    else
        *dir = target;
}

/**
 * @brief 自定义控制器控制龙门架
 * @param gantry 龙门架实例指针
 */
static void Gantry_Controller_Control(GantryInstance *gantry)
{
    static RC_ctrl_t last_controller;
    static float target_x, last_target_x;
    static uint8_t last_sw;

    if (memcmp(&last_controller, &controller, sizeof(RC_ctrl_t)) != 0) // 在收到下一次数据后
    {
        memcpy(&last_controller, &controller, sizeof(RC_ctrl_t));
        last_target_x = target_x;
    }

    /* 按下遥感后,记当前位置为初始位置 */
    if (controller.rc.switch_left != last_sw)
    {
        last_gantry_x = gantry->Gantry_ctrl_cmd.x;
    }
    last_sw = controller.rc.switch_left;

    target_x = last_gantry_x - controller.rc.rocker_l_ * 50000;

    Gantry_Controller_Limit_Speed(&gantry->Gantry_ctrl_cmd.x, target_x, fabs(target_x - last_target_x) / 200);
}

/**
 * @brief 键鼠控制龙门架
 * @param gantry 龙门架结构体指针
 */
static void Gantry_Keyboard_Control(GantryInstance *gantry)
{
    if (gantry->keyboard == NULL) return;

    if (gantry->keyboard->ctrl && !gantry->keyboard->shift) // 预防按键冲突
    {
        // 抬升
        if (gantry->keyboard->r) // 升
            gantry->Gantry_ctrl_cmd.z += gantry->Gantry_param.lift_sens_keyboard;
        else if (gantry->keyboard->f) // 降
            gantry->Gantry_ctrl_cmd.z -= gantry->Gantry_param.lift_sens_keyboard;

        // 前伸
        if (gantry->keyboard->w) // 伸出
            gantry->Gantry_ctrl_cmd.y += gantry->Gantry_param.stretch_sens_keyboard;
        else if (gantry->keyboard->s) // 收回
            gantry->Gantry_ctrl_cmd.y -= gantry->Gantry_param.stretch_sens_keyboard;

        // 横移
        if (gantry->keyboard->a) // 左移
            gantry->Gantry_ctrl_cmd.x -= gantry->Gantry_param.sidesway_sens_keyboard;
        else if (gantry->keyboard->d) // 右移
            gantry->Gantry_ctrl_cmd.x += gantry->Gantry_param.sidesway_sens_keyboard;
    }

    // 开启自定义控制器时记录当前位置，之后自定义控制器以该位置做相对偏移控制
    static uint8_t last_st;
    if (last_st == 0 && gantry->Gantry_ctrl_cmd.controller_st == 1)
    {
        last_gantry_x = gantry->Gantry_ctrl_cmd.x;
    }
    last_st = gantry->keyboard->g;

    // 是否启用自定义控制器
    if (gantry->Gantry_ctrl_cmd.controller_st == 1)
        Gantry_Controller_Control(gantry);
}

/**
 * @brief 遥控模式控制龙门架
 * @param gantry 龙门架结构体指针
 */
static void Gantry_Remote_Control(GantryInstance *gantry)
{
    if (gantry->remote_data == NULL) return; // 安全检查

    gantry->Gantry_ctrl_cmd.x += gantry->remote_data->rc.rocker_r_ * gantry->Gantry_param.sidesway_sens_remote;
    gantry->Gantry_ctrl_cmd.y += gantry->remote_data->rc.rocker_r1 * gantry->Gantry_param.stretch_sens_remote;
    gantry->Gantry_ctrl_cmd.z += gantry->remote_data->rc.rocker_l1 * gantry->Gantry_param.lift_sens_remote;
}

/**
 * @brief 判断龙门架控制模式并进行相应控制
 * @param gantry 龙门架实例指针
 */
static void Gantry_Control(GantryInstance *gantry)
{
    switch (gantry->Gantry_ctrl_cmd.Gantry_mode)
    {
    case GANTRY_MODE_POWER_OFF:
    case GANTRY_MODE_LOCK:
        break;
    case GANTRY_MODE_CONTROL_REMOTE:
        Gantry_Remote_Control(gantry);
        break;
    case GANTRY_MODE_CONTROL_PC:
        Gantry_Keyboard_Control(gantry);
        break;
    default:
        break;
    }
}

/**
 * @brief 龙门架PID计算(角度环与速度环串级计算)
 * @param gantry 龙门架实例指针
 */
static void Gantry_Pid_Calculate(GantryInstance *gantry)
{
    // 注意：这里需要根据实际的PID控制器实现进行调整
    // 目前只是示例代码

    // 抬升电机PID计算
    if (gantry->lift_motor != NULL) {
        // 使用角度环控制
        DJIMotorSetPIDRef(gantry->lift_motor[0].motor, gantry->lift_motor[0].target_ecd);
        DJIMotorSetPIDRef(gantry->lift_motor[1].motor, gantry->lift_motor[1].target_ecd);
    }

    // 前伸电机PID计算
    if (gantry->stretch_motor != NULL) {
        DJIMotorSetPIDRef(gantry->stretch_motor[0].motor, gantry->stretch_motor[0].target_ecd);
        DJIMotorSetPIDRef(gantry->stretch_motor[1].motor, gantry->stretch_motor[1].target_ecd);
    }

    // 横移电机PID计算
    if (gantry->sidesway_motor.motor != NULL) {
        DJIMotorSetPIDRef(gantry->sidesway_motor.motor, gantry->sidesway_motor.target_ecd);
    }
}

/**
 * @brief 使用pid计算结果控制电机转动
 * @param gantry 龙门架实例指针
 */
static void Gantry_Can_Cmd(GantryInstance *gantry)
{
    uint8_t stop = (gantry->Gantry_ctrl_cmd.Gantry_mode == GANTRY_MODE_POWER_OFF);
    // 抬升电机
    for (int i = 0; i < 2; i++) {
        if (gantry->lift_motor[i].motor) {
            if (stop)
                DJIMotorStop(gantry->lift_motor[i].motor);
            else
                DJIMotorEnable(gantry->lift_motor[i].motor);
        }
    }
    // 前伸电机
    for (int i = 0; i < 2; i++) {
        if (gantry->stretch_motor[i].motor) {
            if (stop)
                DJIMotorStop(gantry->stretch_motor[i].motor);
            else
                DJIMotorEnable(gantry->stretch_motor[i].motor);
        }
    }
    // 横移电机
    if (gantry->sidesway_motor.motor) {
        if (stop)
            DJIMotorStop(gantry->sidesway_motor.motor);
        else
            DJIMotorEnable(gantry->sidesway_motor.motor);
    }
}

/**
 * @brief 电控限位
 * @param gantry 龙门架结构体指针
 */
static void Gantry_Limit(GantryInstance *gantry)
{
    static int32_t last_x, last_y, last_z;

    /// 当抬升没有抬到一定高度的时候，不允许前伸过三角，防止结构把电线卡断
    // 当前伸下的拖链越过三角时，不允许后退
    // 当前伸下的拖链处于三角附近时，不允许下降太低
    if (gantry->Gantry_ctrl_cmd.z < 2200)
    {
        if (gantry->Gantry_ctrl_cmd.y > 2200 && gantry->Gantry_ctrl_cmd.y < 4000 && last_y < gantry->Gantry_ctrl_cmd.y)
            gantry->Gantry_ctrl_cmd.y = 2200;
        else if (gantry->Gantry_ctrl_cmd.y < 11500 && gantry->Gantry_ctrl_cmd.y > 9000 && last_y > gantry->Gantry_ctrl_cmd.y)
            gantry->Gantry_ctrl_cmd.y = 11500;

        if (gantry->Gantry_ctrl_cmd.y > 2200 && gantry->Gantry_ctrl_cmd.y < 11500 && last_z > gantry->Gantry_ctrl_cmd.z)
            gantry->Gantry_ctrl_cmd.z = 2100;
    }

    // 抬升
    if (gantry->Gantry_ctrl_cmd.z <= 0)
        gantry->Gantry_ctrl_cmd.z = 0;
    else if (gantry->Gantry_ctrl_cmd.z >= gantry->Gantry_param.GANTRY_MAX_Z)
        gantry->Gantry_ctrl_cmd.z = gantry->Gantry_param.GANTRY_MAX_Z;

    // 前伸
    if (gantry->Gantry_ctrl_cmd.y <= 0)
        gantry->Gantry_ctrl_cmd.y = 0;
    else if (gantry->Gantry_ctrl_cmd.y >= gantry->Gantry_param.GANTRY_MAX_Y)
        gantry->Gantry_ctrl_cmd.y = gantry->Gantry_param.GANTRY_MAX_Y;

    // 横移
    if (gantry->Gantry_ctrl_cmd.x <= 0)
        gantry->Gantry_ctrl_cmd.x = 0;
    else if (gantry->Gantry_ctrl_cmd.x >= gantry->Gantry_param.GANTRY_MAX_X)
        gantry->Gantry_ctrl_cmd.x = gantry->Gantry_param.GANTRY_MAX_X;


    // if (IS_KEY_EXIT_SIDEWAY_LEFT) // 如果碰到了横移限位开关(左)
    // {
    //     init_sidesway_ecd = gantry->motor_sidesway.motor->measure.ecd;
    //     gantry->motor_sidesway.motor->measure.cnt = 0;
    //     if (gantry->x < last_x)
    //         gantry->x = 1;
    // }
    // else if (IS_KEY_EXIT_SIDEWAY_RIGHT) // 如果碰到了横移限位开关(右)
    //     max_x = gantry->x - 10;

    // if (gantry->x > max_x && gantry->x > last_x)
    //     gantry->x = max_x;

    last_x = gantry->Gantry_ctrl_cmd.x;
    last_z = gantry->Gantry_ctrl_cmd.z;
    last_y = gantry->Gantry_ctrl_cmd.y;
}

/**
 * @brief 根据位置矢量解算出电机转动的圈数
 * @param gantry 龙门架结构体指针
 */
static void Gantry_Position_Calculate(GantryInstance *gantry) {
    // 抬升 - 两个电机，方向相反
    gantry->lift_motor[0].target_ecd = init_lift_ecd[0] - gantry->Gantry_ctrl_cmd.z * gantry->Gantry_param.position_ecd_ratio;
    gantry->lift_motor[1].target_ecd = init_lift_ecd[1] + gantry->Gantry_ctrl_cmd.z * gantry->Gantry_param.position_ecd_ratio;

    // 前伸 - 两个电机，方向相反
    gantry->stretch_motor[0].target_ecd = init_stretch_ecd[0] - gantry->Gantry_ctrl_cmd.y * gantry->Gantry_param.position_ecd_ratio;
    gantry->stretch_motor[1].target_ecd = init_stretch_ecd[1] + gantry->Gantry_ctrl_cmd.y * gantry->Gantry_param.position_ecd_ratio;

    // 横移 - 一个电机
    gantry->sidesway_motor.target_ecd = init_sidesway_ecd - gantry->Gantry_ctrl_cmd.x * gantry->Gantry_param.position_ecd_ratio;
}
