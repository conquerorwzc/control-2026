#include "shoot.h"

#include "bsp_dwt.h"
#include "user_lib.h"

static uint8_t idx = 0;  // register idx,是该文件的全局索引,在注册时使用
static ShootInstance* shoot[SHOOT_CNT_MAX] = {NULL};
static Shoot_Ctrl_Cmd_s* shoot_ctrl_cmd[SHOOT_CNT_MAX];

static float one_bullet_delta_angle;
static float reduction_ratio_loader;
static float loader_direction;  // 实际上应该修改loader_config就可以了，但是角度为最外环似乎有bug？？，先打个补丁，后续做修改

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */

static float loader_set = 0;
static float friction_set = 0;
static int frictin_num[SHOOT_CNT_MAX];  // 用于动态初始化每个实例的摩擦轮电机数量，类似于idx的用法

// 波弹盘位置初始化标志
//  static uint8_t loader_position_init=0;

static float hibernate_time = 0, dead_time = 0;

ShootInstance* ShootInit(Shoot_Init_Config_s* shoot_init_config) {
  size_t size = sizeof(ShootInstance) + shoot_init_config->shoot_param.friction_num * sizeof(DJIMotorInstance*);
  ShootInstance* shoot_instance = (ShootInstance*)zmalloc(sizeof(size));

  one_bullet_delta_angle = shoot_init_config->shoot_param.one_bullet_delta_angle;
  reduction_ratio_loader = shoot_init_config->shoot_param.reduction_ratio_loader;
  loader_direction = shoot_init_config->shoot_param.loader_direction;

  shoot_init_config->loader_motor_config.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
  shoot_init_config->loader_motor_config.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
  shoot_init_config->loader_motor_config.controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
  shoot_init_config->loader_motor_config.controller_setting_init_config.close_loop_type = SPEED_LOOP | ANGLE_LOOP;

  for (int i = 0; i < shoot_init_config->shoot_param.friction_num; i++) {
    shoot_instance->friction_motor[i] = DJIMotorInit(&shoot_init_config->friction_motor_config[i]);
    frictin_num[idx]++;
    ;
  }
  shoot_instance->loader_motor = DJIMotorInit(&shoot_init_config->loader_motor_config);

  shoot[idx] = shoot_instance;
  shoot_ctrl_cmd[idx] = &shoot_instance->shoot_ctrl_cmd;  // 在运行时初始化指针
  idx++;
  return shoot_instance;
}

/* 机器人发射机构控制核心任务 */
void ShootTask() {
  for (size_t i = 0; i < idx; ++i) {  // 遍历实例去控制，目前只有shoot这个写法，因为之前哨兵是双枪管的，时代的眼泪
    if (shoot_ctrl_cmd[i]->shoot_mode == SHOOT_OFF) {
      for (int j = 0; j < frictin_num[i]; j++) DJIMotorStop(shoot[i]->friction_motor[j]);
      DJIMotorStop(shoot[i]->loader_motor);
    } else  // 恢复运行
    {
      for (int j = 0; j < frictin_num[i]; j++) DJIMotorEnable(shoot[i]->friction_motor[j]);
      DJIMotorEnable(shoot[i]->loader_motor);

      for (int j = 0; j < frictin_num[i]; j++) DJIMotorSetPIDRef(shoot[i]->friction_motor[j], friction_set);
      DJIMotorSetPIDRef(shoot[i]->loader_motor, loader_set);
    }
    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    if (hibernate_time + dead_time > DWT_GetTimeline_ms()) continue;

    if (shoot[i]->loader_motor->motor_controller.speed_PID.ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR) {
      shoot[i]->loader_motor->motor_controller.speed_PID.ERRORHandler.ERRORType = PID_ERROR_NONE;  // 清空标志位
      shoot_ctrl_cmd[i]->load_mode = LOAD_REVERSE;
    }

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_ctrl_cmd[i]->load_mode) {
      // 停止拨盘
      case LOAD_STOP:
        DJIMotorOuterLoop(shoot[i]->loader_motor, SPEED_LOOP);  // 切换到速度环
        loader_set = 0;                                         // 同时设定参考值为0,这样停止的速度最快
        break;
        // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
      case LOAD_1_BULLET:                                       // 激活能量机关/干扰对方用,英雄用.
        DJIMotorOuterLoop(shoot[i]->loader_motor, ANGLE_LOOP);  // 切换到角度环
        loader_set = shoot[i]->loader_motor->measure.total_angle +
                     one_bullet_delta_angle * reduction_ratio_loader * loader_direction;  // 控制量增加一发弹丸的角度
        hibernate_time = DWT_GetTimeline_ms();                                            // 记录触发指令的时间
        dead_time = 250;                                                                  // 完成1发弹丸发射的时间
        break;
        // 连发模式,对位置闭环,射频根据dead_time改变；原版是速度闭环，可能会更柔和一些？
      case LOAD_BURSTFIRE:
        DJIMotorOuterLoop(shoot[i]->loader_motor, ANGLE_LOOP);  // 切换到角度环
        loader_set = shoot[i]->loader_motor->measure.total_angle +
                     one_bullet_delta_angle * reduction_ratio_loader * loader_direction;  // 控制量增加一发弹丸的角度
        hibernate_time = DWT_GetTimeline_ms();                                            // 记录触发指令的时间
        dead_time = 300;                                                                  // 弹频
        break;
        // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
        // 也有可能需要从switch-case中独立出来
      case LOAD_REVERSE:
        DJIMotorOuterLoop(shoot[i]->loader_motor, ANGLE_LOOP);  // 切换到角度环
        loader_set = shoot[i]->loader_motor->measure.total_angle -
                     one_bullet_delta_angle * reduction_ratio_loader * loader_direction;
        // 控制量增加一发弹丸的角度
        hibernate_time = DWT_GetTimeline_ms();  // 记录触发指令的时间
        dead_time = 1000;
        // ...
        break;
      default:
        while (1);  // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }
    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_ctrl_cmd[i]->friction_mode == FRICTION_ON) {
      // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
      switch (shoot_ctrl_cmd[i]->bullet_speed) {
        case SMALL_AMU_15:
          friction_set = 20000;
          break;
        case SMALL_AMU_18:
          friction_set = 20000;
          break;
        case SMALL_AMU_30:
          friction_set = 20000;
          break;
        default:  // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
          friction_set = 20000;
          break;
      }
    } else  // 关闭摩擦轮
    {
      friction_set = 0;
    }
  }

  // Todo: 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
}
