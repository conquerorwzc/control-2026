#include "shoot.h"
#include "rm_referee.h"
#include "bsp_dwt.h"
#include "user_lib.h"

static uint8_t idx = 0;  // register idx,是该文件的全局索引,在注册时使用
static ShootInstance* shoot;
static Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd;

// 添加PID控制器用于弹速控制

static float one_bullet_delta_angle;
static float reduction_ratio_loader;
static float
    loader_direction;  // 实际上应该修改loader_config就可以了，但是角度为最外环似乎有bug？？，先打个补丁，后续做修改
static float friction_speed;
static float friction_coefficients[FRICTION_NUM];
/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */

static float loader_set = 0;       // 波胆盘速度
static float friction_set = 0;     // 摩擦轮速度
static float actual_bullet_speed = 0.0f;  // 调试用，后续从裁判系统中获取
// 波弹盘位置初始化标志
//  static uint8_t loader_position_init=0;

static float hibernate_time = 0, dead_time = 0;
static float deadtime_burstfire;
static float deadtime_onebullet;
static float target_speed;
static float bullet_speed_adjustment;

ShootInstance* ShootInit(Shoot_Init_Config_s* shoot_init_config) {
  ShootInstance* shoot_instance = (ShootInstance*)zmalloc(sizeof(ShootInstance));

  one_bullet_delta_angle = shoot_init_config->shoot_param.one_bullet_delta_angle;
  reduction_ratio_loader = shoot_init_config->shoot_param.reduction_ratio_loader;
  loader_direction = shoot_init_config->shoot_param.loader_direction;
  friction_speed = shoot_init_config->shoot_param.friction_speed;
  deadtime_burstfire = shoot_init_config->shoot_param.deadtime_burstfire;
  deadtime_onebullet = shoot_init_config->shoot_param.deadtime_onebullet;
  target_speed = shoot_init_config->shoot_param.target_speed;
  bullet_speed_adjustment = shoot_init_config->shoot_param.bullet_speed_adjustment;
  //actual_bullet_speed = &GetReferee()->ShootData.bullet_speed;
  // 初始化弹速控制PID参数

  // 初始化弹速控制PID控制器

  for (int i = 0; i < FRICTION_NUM; i++) {
    friction_coefficients[i] = shoot_init_config->shoot_param.friction_coefficients[i];
  }
  for (int i = 0; i < FRICTION_NUM; i++) {
    shoot_instance->friction_motor[i] = DJIMotorInit(&shoot_init_config->friction_motor_config[i]);
  }
  shoot_instance->loader_motor = DJIMotorInit(&shoot_init_config->loader_motor_config);

  shoot = shoot_instance;
  shoot_ctrl_cmd = &shoot_instance->shoot_ctrl_cmd;  // 在运行时初始化指针
  idx++;
  return shoot_instance;
}
/**
 * @brief 弹速控制函数，根据实际弹速与目标弹速的差异动态调整摩擦轮转速,后续实际弹速从裁判系统中获取
 */
void ShootBulletSpeedControl(void) {
  // 计算弹速误差

  float speed_error = target_speed - actual_bullet_speed;

  // 将误差乘以系数后加到基础摩擦轮速度上
  friction_speed = friction_speed + speed_error * bullet_speed_adjustment;
}

// ... existing code ...

/* 机器人发射机构控制核心任务 */
void ShootTask() {  // 遍历实例去控制，目前只有shoot这个写法，因为之前哨兵是双枪管的，时代的眼泪
  actual_bullet_speed= GetReferee()->ShootData.initial_speed;

  if (shoot_ctrl_cmd->shoot_mode == SHOOT_OFF) {
   // for (int j = 0; j < FRICTION_NUM; j++) DJIMotorStop(shoot->friction_motor[j]);
    //friction_set=0;
    DJIMotorStop(shoot->loader_motor);
    for (int j = 0; j < FRICTION_NUM; j++)
      DJIMotorSetPIDRef(shoot->friction_motor[j], 0);
  } else  // 恢复运行
  {
    for (int j = 0; j < FRICTION_NUM; j++) DJIMotorEnable(shoot->friction_motor[j]);
    DJIMotorEnable(shoot->loader_motor);

    DJIMotorSetPIDRef(shoot->loader_motor, loader_set);
    // 使用根据机器人类型设置的摩擦轮系数
    for (int i = 0; i < FRICTION_NUM; i++) {
      DJIMotorSetPIDRef(shoot->friction_motor[i], friction_coefficients[i] * friction_set);
    }
  }
  // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
  if (hibernate_time + dead_time > DWT_GetTimeline_ms()) return;
  ;


  if (shoot->loader_motor->motor_controller.speed_PID.ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR&&shoot->loader_motor->motor_controller.angle_PID.ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR) {
    shoot->loader_motor->motor_controller.speed_PID.ERRORHandler.ERRORType = PID_ERROR_NONE;  // 清空标志位
    shoot->loader_motor->motor_controller.angle_PID.ERRORHandler.ERRORType = PID_ERROR_NONE;
    shoot_ctrl_cmd->load_mode = LOAD_REVERSE;
  }
  // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
  switch (shoot_ctrl_cmd->load_mode) {
    // 停止拨盘
    case LOAD_STOP:
      DJIMotorOuterLoop(shoot->loader_motor, SPEED_LOOP);  // 切换到速度环
      loader_set = 0;                                      // 同时设定参考值为0,这样停止的速度最快
      break;
      // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:  // 激活能量机关/干扰对方用,英雄用.
      //ShootBulletSpeedControl();
      DJIMotorOuterLoop(shoot->loader_motor, ANGLE_LOOP);  // 切换到角度环
      loader_set = shoot->loader_motor->measure.total_angle +
                   one_bullet_delta_angle * reduction_ratio_loader * loader_direction;  // 控制量增加一发弹丸的角度
      hibernate_time = DWT_GetTimeline_ms();                                            // 记录触发指令的时间
      dead_time = deadtime_onebullet;                                                   // 完成1发弹丸发射的时间
      break;
      // 连发模式,对位置闭环,射频根据dead_time改变；原版是速度闭环，可能会更柔和一些？
    case LOAD_BURSTFIRE:
      //ShootBulletSpeedControl();
      DJIMotorOuterLoop(shoot->loader_motor, ANGLE_LOOP);  // 切换到角度环
      loader_set = shoot->loader_motor->measure.total_angle +
                   one_bullet_delta_angle * reduction_ratio_loader * loader_direction;  // 控制量增加一发弹丸的角度
      //ShootBulletSpeedControl();
      hibernate_time = DWT_GetTimeline_ms();                                            // 记录触发指令的时间
      dead_time = deadtime_burstfire;                                                   // 弹频
      break;
      // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
      // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
      DJIMotorOuterLoop(shoot->loader_motor, ANGLE_LOOP);  // 切换到角度环
      loader_set =
          shoot->loader_motor->measure.total_angle - one_bullet_delta_angle * reduction_ratio_loader * loader_direction;
      // 控制量增加一发弹丸的角度
      hibernate_time = DWT_GetTimeline_ms();  // 记录触发指令的时间
      dead_time = deadtime_onebullet;
      // ...
      break;
    default:
      while (1);  // 未知模式,停止运行,检查指针越界,内存溢出等问题
  }
  // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
  if (shoot_ctrl_cmd->friction_mode == FRICTION_ON) {
    // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
    friction_set = friction_speed;
  } else  // 关闭摩擦轮
  {
    friction_set = 0;
  }

  // Todo: 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
}
