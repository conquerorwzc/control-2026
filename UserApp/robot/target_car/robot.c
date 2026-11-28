/**
 * @file robot.c
 * @brief target_car: 老遥控器(DT7/DBUS)控制一个 6020(GM6020, CAN1/ID4) 以恒定角速度匀速旋转
 *
 * 遥控器右侧拨杆(rc.switch_right) -> 按"拨杆物理位置"定义的档位:
 *   下档 -> 电机失能
 *   中档 -> 0.4 rad/s 匀速旋转
 *   上档 -> 0.7 rad/s 匀速旋转
 *   遥控器离线 -> 电机失能(安全保护)
 *   注意: 该遥控器拨杆的"上/下"与驱动的 RC_SW_UP/RC_SW_DOWN 是反的, 所以下面的 switch 里是对调的。
 *
 * @attention 单位! dji_motor 速度环的反馈 measure.speed_aps 单位是 度/秒(deg/s),
 *            因此速度环参考值也必须用 度/秒 下发, 而不是 rad/s:
 *              0.4 rad/s = 0.4 * 180/pi = 22.92 deg/s
 *              0.7 rad/s = 0.7 * 180/pi = 40.11 deg/s
 *            所以这里统一用 RAD_2_DEGREE 换算后再调用 DJIMotorSetPIDRef()。
 *
 * @note 档位一律按"拨杆物理位置"来写: 下档=失能 / 中档=0.4rad/s / 上档=0.7rad/s。
 *
 * 左拨杆(rc.switch_left):
 *   中档 -> 正弦变速"小陀螺": 2 ~ 15 rad/s, 周期2s, 优先级高于右拨杆的定速档
 *   其它档/遥控器离线 -> 走右拨杆的定速逻辑
 */
#include "robot.h"
#include "controller.h"
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "general_def.h"
#include "remote_control.h"
#include "robot_config.h"
#include "usart.h"

static DJIMotorInstance* gm6020_instance = NULL;  // 6020 电机实例
static RC_ctrl_t* rc_data = NULL;                 // 老遥控器数据
static uint8_t gm6020_enabled = 0;                // 电机使能状态, 用于处理使能/失能边沿

/**
 * @brief 左拨杆中档的"小陀螺"速度参考: 正弦变速
 *        ω(t) = 8.5 + 6.5 * sin(2π*t/2)  rad/s  -> 最小2, 最大15, 周期2s
 * @note  相位用 DWT 的绝对时间轴, 所以进出该模式时相位是连续的, 不会有跳变
 * @return float 目标角速度, 单位 rad/s
 */
static float SpinSpeedRef() {
  const float offset = (SPIN_SPEED_MAX_RAD_S + SPIN_SPEED_MIN_RAD_S) * 0.5f;  // 8.5 rad/s
  const float amplitude = (SPIN_SPEED_MAX_RAD_S - SPIN_SPEED_MIN_RAD_S) * 0.5f;  // 6.5 rad/s
  const float t = DWT_GetTimeline_s();                                         // 运行时间, 单位 s
  return offset + amplitude * sinf(PI2 * t / SPIN_PERIOD_S);
}

void RobotInit() {
#ifdef STM32F407xx
  rc_data = RemoteControlInit(&huart3);  // 老遥控器(DT7), C板上使用USART3
#elifdef STM32H723xx
  rc_data = RemoteControlInit(&huart5);
#endif

  gm6020_instance = DJIMotorInit(&GM6020_config);
  DJIMotorStop(gm6020_instance);  // 上电默认失能, 等待遥控器指令
}

void RobotTask() {
  float target_speed_rad_s = 0.0f;  // 目标角速度, 单位 rad/s
  uint8_t need_enable = 1;

  if (!RemoteControlIsOnline()) {
    need_enable = 0;  // 遥控器掉线, 直接失能
  } else {
    switch (rc_data[TEMP].rc.switch_right) {  // 注意用 TEMP 取当前帧数据
      case RC_SW_DOWN:  // 拨杆物理"下档"(该遥控器上下与驱动值相反): 失能
        need_enable = 0;
        break;
      case RC_SW_MID:  // 拨杆中档: 0.4 rad/s
        target_speed_rad_s = TARGET_SPEED_MID_RAD_S;
        break;
      case RC_SW_UP:  // 拨杆物理"上档": 0.7 rad/s
        target_speed_rad_s = TARGET_SPEED_HIGH_RAD_S;
        break;
      default:  // RC_SW_OFF, 开关/遥控器数据无效
        need_enable = 0;
        break;
    }

    // 左拨杆打到中档: 正弦变速"小陀螺"(优先级高于右拨杆的定速档)
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) {
      need_enable = 1;
      target_speed_rad_s = SpinSpeedRef();
    }
  }

  if (need_enable) {
    if (!gm6020_enabled) {  // 由失能切到使能: 先清掉上一次的积分残留, 避免使能瞬间冲高
      PIDClear(&gm6020_instance->motor_controller.speed_PID);
      DJIMotorEnable(gm6020_instance);
      gm6020_enabled = 1;
    }
    // rad/s -> deg/s (0.4rad/s = 22.92deg/s, 0.7rad/s = 40.11deg/s)
    DJIMotorSetPIDRef(gm6020_instance, target_speed_rad_s * RAD_2_DEGREE);
  } else {
    if (gm6020_enabled) {
      DJIMotorStop(gm6020_instance);  // 失能: 发送给电调的指令直接被置零
      gm6020_enabled = 0;
    }
    DJIMotorSetPIDRef(gm6020_instance, 0.0f);  // 参考值清零, 让积分自然退出
  }
}
