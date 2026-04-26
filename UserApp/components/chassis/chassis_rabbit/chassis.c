

#include "chassis.h"
#include "external_imu/external_imu.h"
#include "arm_math.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "robot_config.h"
#include "user_lib.h"
static ChassisInstance* chassis;
static Chassis_Ctrl_Cmd_s* chassis_ctrl_cmd;  // 声明但不初始化
static Chassis_Param_s chassis_param;          // 声明为静态局部变量
/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;     // 将云台系的速度投影到底盘
static referee_info_t *referee_data;
static float vt_lf, vt_rf, vt_lb, vt_rb;  // 底盘速度解算后的临时输出,待进行限幅
static float lf_radius;
static float rf_radius;
static float lb_radius;
static float rb_radius;
static float follow_angle=0;//底盘跟随目标角度
static float normal_follow_tick;//记录最后一次follow前方的时间戳
static float reverse_follow_tick;//记录最后一次follow后方的时间戳
static PIDInstance follow_pid;
static float k0,k1,k2,k3,k4,k5;       //中科大的功率模型
static float power;

// 左右腿目标位置
static float target_position_left = 0.0f;
static float target_position_right = 0.0f;

// 左右腿当前位置（静态变量，保持状态）
static float leg_current_position_left = 0.0f;
static float leg_current_position_right = 0.0f;
// 调试监控变量
static float left_torque_feedforward = 0.0f;
static float right_torque_feedforward = 0.0f;
static float pos_l_currentx = 0.0f;
static float pos_l_currenty = 0.0f;
static float debug_pos_l=0.0f;
static float debug_pos_r=0.0f;
static float debug_force_l=0.0f;
static float debug_force_r=0.0f;
// 雅可比力臂监控 (Jy)
static float debug_Jy_l = 0.0f;
static float debug_Jy_r = 0.0f;
// 垂直支撑力监控 (Fy)
static float debug_Fy_total = 0.0f;
// 定义常量
#define G 9.81f
#define NUM_REAR_LEGS 2.0f // 后腿数量
#define PI 3.1415926535f
// ==================== 机器人物理与机构参数 ====================
//  整车动态负载参数 (基于实测双秤垫高法解算)
const float M_TOTAL = 32.00f;  // 整车总质量 (kg)
const float L_F     = 0.2088f; // 重心距前轴水平距离 (m)
const float H_CG    = 0.250f;  // 重心垂直高度 (m)
const float M_LEG   = 2.0f;    // 腿重量（kg)
const float L_FRONT_TO_MOTOR = 0.40f; //前轴到后关节电机的水平距离（m）
const float M_THIGH = 0.2f;
const float L2_HALF = 0.10522f;
const float M_WHEEL = 1.8f;
// ==================== 提取到外部的物理几何参数 ====================
//连杆参数
// 在全局坐标系下 (X正向朝车尾，Y正向朝地)
const float L1_X  = -0.06782f;  // D点在电机前方 -> 负值
const float L1_Y  = -0.06782f;  // D点在电机上方 -> 负值
const float L2    = 0.21044f;
const float L3    = 0.05923f;
const float L4    = 0.20851f;
const float L_EXT = 0.20256f;
// theta=0 时，主动杆 L2 绝对水平指向车头；向下压时 theta 为正。
#define LEFT_MOTOR_HORIZON_OFFSET  (-2.75687f) // 左腿绝对水平时的编码器弧度
#define RIGHT_MOTOR_HORIZON_OFFSET 2.52379f  // 右腿绝对水平时的编码器弧度
// 电机旋转方向系数 (如果往下压时编码器数值减小，则填 -1.0f，增大填 1.0f)
#define LEFT_MOTOR_DIR   1.0f
#define RIGHT_MOTOR_DIR  (-1.0f)
// 在文件顶部定义位置缓变速率和极限角度 (均为标准数学弧度)
#define LEG_RAMP_RATE       0.0005f  // 腿长变化速度 (弧度/ms)，数值越小动作越慢
#define UPPER_LIMIT_ANGLE   0.390f   // 上方机械限位角度 (几乎水平)
#define LOWER_LIMIT_ANGLE   1.0f   // 下方最大伸展角度 (防止顶死或奇异点)
                                   //单位都为弧度，车辆坐标系
// 轮子坐标结构体
typedef struct {
  float x;
  float y;
  uint8_t is_valid;
} WheelPos_t;
// ==============================================================
float rad_to_deg(float encoder_val) {
  return encoder_val * (180.0f / PI);
}
static void CalculateEulerCompensation(ChassisInstance* chassis, float* compensation_x, float* compensation_y) {
  if (chassis->chassis_external_imu != NULL) {
    // 使用外部IMU的欧拉角数据计算补偿
    float roll = chassis->chassis_external_imu->pitch;
    float pitch = chassis->chassis_external_imu->roll;

    // 根据欧拉角计算腿部位置补偿
    // 这里假设一定的补偿系数，可根据实际效果调整
    float roll_compensation_factor = 0.005f;  // 横滚补偿系数
    float pitch_compensation_factor = 0.01f; // 俯仰补偿系数

    *compensation_x = pitch * pitch_compensation_factor;  // 俯仰影响前后腿
    *compensation_y = roll * roll_compensation_factor;    // 横滚影响左右腿差异
  } else {
    // 如果没有IMU数据，补偿值为0
    *compensation_x = 0.0f;
    *compensation_y = 0.0f;
  }
}
/**
 * @brief 飞坡补偿模式控制
 */
static void SlopeCompensationControl() {
  if (chassis->chassis_external_imu != NULL) {
    // 计算基于欧拉角的补偿
    float compensation_x, compensation_y;
    CalculateEulerCompensation(chassis, &compensation_x, &compensation_y);
    //限制最大补偿角度为0.2
    if (fabsf(compensation_x)>0.2) {
      compensation_x = (compensation_x > 0) ? 0.2f : -0.2f;
    }
    if (fabsf(compensation_y)>0.2) {
      compensation_y = (compensation_y > 0) ? 0.2f : -0.2f;
    }
    // 应用补偿到腿部目标位置
    float target_left = LEFT_LEG_MOTOR_CRUISE_POSITION - compensation_x+ compensation_y;
    float target_right = RIGHT_LEG_MOTOR_CRUISE_POSITION  + compensation_x+ compensation_y;
    if (target_left < LEFT_LEG_MOTOR_NORMAL_POSITION) {
      target_left = LEFT_LEG_MOTOR_NORMAL_POSITION;
    }
    if (target_right > RIGHT_LEG_MOTOR_NORMAL_POSITION) {
      target_right = RIGHT_LEG_MOTOR_NORMAL_POSITION;
    }
    // 限制最大位置为KIKE位置
    if (target_left> LEFT_LEG_MOTOR_KIKE_POSITION) {
      target_left = LEFT_LEG_MOTOR_KIKE_POSITION;
    }
    if (target_right < RIGHT_LEG_MOTOR_KIKE_POSITION) {
      target_right = RIGHT_LEG_MOTOR_KIKE_POSITION;
    }
    // 设置腿部电机目标
    DMMotorSetPIDRef(chassis->leg_motor[0], target_left);
    DMMotorSetPIDRef(chassis->leg_motor[1], target_right);
  }
}

/**
 * @brief 腿部正运动学解算 (原生坐标系，无翻转补丁)
 * @param theta 电机标准下压角(弧度，水平指向车头为0，向下压为正)
 */
static WheelPos_t Kinematics_Calc_Wheel(float theta) {
  WheelPos_t pos = {0, 0, 0};

  // 1. 主动杆 B 点坐标 (腿往前伸，所以 X 带有天然负号)
  float B_x = -L2 * arm_cos_f32(theta);
  float B_y = L2 * arm_sin_f32(theta);

  // 2. 计算 B 点到 D 点的向量与距离
  float dx = L1_X - B_x;
  float dy = L1_Y - B_y;
  float d_sq = dx*dx + dy*dy;
  float d = sqrtf(d_sq);

  // 3. 物理极限安全保护 (防止机构扯断或折叠卡死)
  if (d > (L3 + L4) || d < fabsf(L3 - L4)) {
    pos.is_valid = 0;
    return pos;
  }

  // 4. 解三角形 B-D-C
  float gamma = atan2f(dy, dx);
  float cos_alpha = (L3*L3 + d_sq - L4*L4) / (2.0f * L3 * d);
  float alpha = acosf(cos_alpha);

  // 5. 决定四连杆折叠方向 (剪刀机构的正确物理姿态)
  // 在当前原生坐标系下，通常用减号；若发现轮子算到了天上，改为加号
  float theta_3 = gamma - alpha;

  // 6. 计算 C 点坐标
  float C_x = B_x + L3 * arm_cos_f32(theta_3);
  float C_y = B_y + L3 * arm_sin_f32(theta_3);

  // 7. 沿着 C-B 连线延长，计算轮轴 W 坐标
  float vec_CB_x = B_x - C_x;
  float vec_CB_y = B_y - C_y;

  // 此时算出的 W_x 天然就是正数(指向车尾)
  pos.x = B_x + (vec_CB_x / L3) * L_EXT;
  pos.y = B_y + (vec_CB_y / L3) * L_EXT;
  pos.is_valid = 1;

  return pos;
}
/**
 * @brief 动态触地干扰观测器 (DOB)
 * @return float 返回足底估计的外部受力 (牛顿 N)
 */
static float Estimate_Contact_Force_Math(float t_real_math, float theta_math, float vel_math, float J_y, float *t_ext_filtered) {

  // 1. 重力期望 (悬空时需向上抬，所以带负号)
  float t_thigh = M_THIGH * G * L2_HALF * arm_cos_f32(theta_math);
  float t_wheel = M_WHEEL * G * J_y;
  float t_expected_air = -(t_thigh + t_wheel);

  // 2. 摩擦力补偿 (直接使用已经归一化好的 vel_math)
  if (vel_math > 0.05f) {
    t_expected_air += 0.2f + 0.05f * vel_math;
  } else if (vel_math < -0.05f) {
    t_expected_air += -0.2f + 0.05f * vel_math;
  }

  // 3. 计算干扰并滤波
  float t_ext_raw = t_real_math - t_expected_air;
  const float ALPHA = 0.05f;
  *t_ext_filtered = ALPHA * t_ext_raw + (1.0f - ALPHA) * (*t_ext_filtered);

  // 4. 折算输出
  float safe_Jy = (J_y < 0.02f) ? 0.02f : J_y;
  float foot_force = (*t_ext_filtered) / safe_Jy;

  return foot_force;
}
static void NewLegControl() {
    static float p_des_math_l = UPPER_LIMIT_ANGLE;
    static float p_des_math_r = UPPER_LIMIT_ANGLE;
    static uint8_t is_first_run = 1;

    // 0. 失能状态检测与无扰切换重置
    if (chassis_ctrl_cmd->chassis_mode == CHASSIS_POWER_OFF ||
        chassis_ctrl_cmd->leg_mode == LEG_DISABLE) {
        is_first_run = 1;
        return;
    }

    // 1. 获取车身 Pitch 仰角 (这里可根据实际情况加滤波)
    float pitch_angle = 0.0f;
    if (chassis->chassis_external_imu != NULL) {
        pitch_angle = (-1.0f * chassis->chassis_external_imu->roll) * 0.01745329f;
    }

    // 2. 读取电机真实的物理状态
    float raw_angle_l = chassis->leg_motor[0]->measure.total_angle;
    float raw_angle_r = chassis->leg_motor[1]->measure.total_angle;
    float theta_math_l = (raw_angle_l - LEFT_MOTOR_HORIZON_OFFSET) * LEFT_MOTOR_DIR;
    float theta_math_r = (raw_angle_r - RIGHT_MOTOR_HORIZON_OFFSET) * RIGHT_MOTOR_DIR;

    float vel_l = chassis->leg_motor[0]->measure.velocity * LEFT_MOTOR_DIR;
    float vel_r = chassis->leg_motor[1]->measure.velocity * RIGHT_MOTOR_DIR;
    float t_real_math_l = chassis->leg_motor[0]->measure.torque * LEFT_MOTOR_DIR;
    float t_real_math_r = chassis->leg_motor[1]->measure.torque * RIGHT_MOTOR_DIR;
    // 清洗数据，归一极性，方便后续数学解算
    // 开机/唤醒无缝衔接
    if (is_first_run) {
        p_des_math_l = theta_math_l;
        p_des_math_r = theta_math_r;
        is_first_run = 0;
    }

    // 3. 目标轨迹平滑生成
    switch (chassis_ctrl_cmd->leg_mode) {
        case LEG_MANUAL_DOWN:
            p_des_math_l -= LEG_RAMP_RATE; p_des_math_r -= LEG_RAMP_RATE;
        break;
        case LEG_MANUAL_UP:
            p_des_math_l += LEG_RAMP_RATE; p_des_math_r += LEG_RAMP_RATE;
        break;
        default:
        break;
    }

    // 限幅保护
    if (p_des_math_l > LOWER_LIMIT_ANGLE) p_des_math_l = LOWER_LIMIT_ANGLE;
    if (p_des_math_l < UPPER_LIMIT_ANGLE) p_des_math_l = UPPER_LIMIT_ANGLE;
    if (p_des_math_r > LOWER_LIMIT_ANGLE) p_des_math_r = LOWER_LIMIT_ANGLE;
    if (p_des_math_r < UPPER_LIMIT_ANGLE) p_des_math_r = UPPER_LIMIT_ANGLE;

    // 4. 全局前馈力学解算
    WheelPos_t pos_l_ff = Kinematics_Calc_Wheel(theta_math_l);
    WheelPos_t pos_r_ff = Kinematics_Calc_Wheel(theta_math_r);


    float l_base_real = L_FRONT_TO_MOTOR + pos_l_ff.x;
    if(l_base_real <= 0.1f) l_base_real = 0.1f;

    float m_load_rear_total = M_TOTAL * ((L_F * arm_cos_f32(pitch_angle) + H_CG * arm_sin_f32(pitch_angle)) / l_base_real);
    float m_load_single = m_load_rear_total / NUM_REAR_LEGS;
    if (m_load_single < 0.0f) m_load_single = 0.0f;

    float force_support = m_load_single * G;
    float F_x_chassis = force_support * arm_sin_f32(pitch_angle);
    float F_y_chassis = force_support * arm_cos_f32(pitch_angle);
    debug_Fy_total=F_y_chassis;


    // ============================== 【左腿独立控制块】 ==============================
    float J_x_l = 0.0f, J_y_l = 0.01f;
    float delta_theta = 0.001f;

    // 运动学求导 (获取平滑 Jy)
    WheelPos_t pos1_l = pos_l_ff;
    WheelPos_t pos2_l = Kinematics_Calc_Wheel(theta_math_l+ delta_theta);
    if (pos1_l.is_valid && pos2_l.is_valid) {
        J_x_l = (pos2_l.x - pos1_l.x) / delta_theta;
        J_y_l = (pos2_l.y - pos1_l.y) / delta_theta;
    }

    // 呼叫触地观测器
  // 声明左腿专属滤波器变量并调用
  static float filter_state_l = 0.0f;
  float contact_force_l = Estimate_Contact_Force_Math(t_real_math_l, theta_math_l, vel_l, J_y_l, &filter_state_l);
  debug_force_l = contact_force_l;
    //施密特触发器 (状态机防抖)
    static uint8_t is_stance_l = 0;
    if (contact_force_l > 20.0f) {
        is_stance_l = 1; // 踩实了
    } else if (contact_force_l < 10.0f) {
        is_stance_l = 0; // 离地腾空
    }
  if (p_des_math_l <= UPPER_LIMIT_ANGLE + 0.02f) {
    is_stance_l = 0;
  }
    // 确定当前状态下的目标参数
    float target_tff_l = 0.0f, target_kp_l = 20.0f, target_kd_l = 2.0f;
    if (is_stance_l) {
        target_tff_l = (F_x_chassis * J_x_l) + (F_y_chassis * J_y_l);
        target_kp_l = 150.0f;
        target_kd_l = 5.0f;
    }
    debug_Jy_l = J_y_l;
    // 核心平滑层：时间维度的一阶低通融合
    static float actual_tff_l = 0.0f, actual_kp_l = 20.0f, actual_kd_l = 2.0f;
    const float BLEND_ALPHA = 0.1f; // 值越小，触地后的刚度升起越柔和
    actual_tff_l += BLEND_ALPHA * (target_tff_l - actual_tff_l);
    actual_kp_l  += BLEND_ALPHA * (target_kp_l  - actual_kp_l);
    actual_kd_l  += BLEND_ALPHA * (target_kd_l  - actual_kd_l);
    //三个发送参数都平滑

    // 限幅与下发
    if (actual_tff_l > 40.0f) actual_tff_l = 40.0f;
    if (actual_tff_l < -40.0f) actual_tff_l = -40.0f;
    left_torque_feedforward = actual_tff_l * LEFT_MOTOR_DIR;
    float p_des_raw_l = (p_des_math_l * LEFT_MOTOR_DIR) + LEFT_MOTOR_HORIZON_OFFSET;
    debug_pos_l=p_des_math_l;
    DMMotorSetMITRef(chassis->leg_motor[0], p_des_raw_l, 0.0f, actual_kp_l, actual_kd_l, left_torque_feedforward);


    // ============================== 【右腿独立控制块】 ==============================
    float J_x_r = 0.0f, J_y_r = 0.01f;

    WheelPos_t pos1_r = pos_r_ff;
    WheelPos_t pos2_r = Kinematics_Calc_Wheel(theta_math_r + delta_theta);
    if (pos1_r.is_valid && pos2_r.is_valid) {
        J_x_r = (pos2_r.x - pos1_r.x) / delta_theta;
        J_y_r = (pos2_r.y - pos1_r.y) / delta_theta;
    }

  // 声明右腿专属滤波器变量并调用

  static float filter_state_r = 0.0f;
  float contact_force_r = Estimate_Contact_Force_Math(t_real_math_r, theta_math_r, vel_r, J_y_r, &filter_state_r);
  debug_force_r = contact_force_r;
    static uint8_t is_stance_r = 0;
    if (contact_force_r > 20.0f) {
        is_stance_r = 1;
    } else if (contact_force_r < 10.0f) {
        is_stance_r = 0;
    }
  if (p_des_math_r <= UPPER_LIMIT_ANGLE + 0.02f) {
    is_stance_r = 0;
  }

    float target_tff_r = 0.0f, target_kp_r = 20.0f, target_kd_r = 2.0f;
    if (is_stance_r) {
        target_tff_r = (F_x_chassis * J_x_r) + (F_y_chassis * J_y_r);
        target_kp_r = 150.0f;
        target_kd_r = 5.0f;
    }
    debug_Jy_r= J_y_r;
    static float actual_tff_r = 0.0f, actual_kp_r = 20.0f, actual_kd_r = 2.0f;
    actual_tff_r += BLEND_ALPHA * (target_tff_r - actual_tff_r);
    actual_kp_r  += BLEND_ALPHA * (target_kp_r  - actual_kp_r);
    actual_kd_r  += BLEND_ALPHA * (target_kd_r  - actual_kd_r);

    if (actual_tff_r > 40.0f) actual_tff_r = 40.0f;
    if (actual_tff_r < -40.0f) actual_tff_r = -40.0f;
    right_torque_feedforward = actual_tff_r * RIGHT_MOTOR_DIR;
    debug_pos_r=p_des_math_r;
    float p_des_raw_r = (p_des_math_r * RIGHT_MOTOR_DIR) + RIGHT_MOTOR_HORIZON_OFFSET;
    DMMotorSetMITRef(chassis->leg_motor[1], p_des_raw_r, 0.0f, actual_kp_r, actual_kd_r, right_torque_feedforward);
}


/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate() {
  vt_lf = -chassis_vx - chassis_vy - chassis_ctrl_cmd->wz * lf_radius;
  vt_rf = -chassis_vx + chassis_vy - chassis_ctrl_cmd->wz  * rf_radius;
  vt_lb = chassis_vx - chassis_vy - chassis_ctrl_cmd->wz  * lb_radius;
  vt_rb = chassis_vx + chassis_vy - chassis_ctrl_cmd->wz  * rb_radius;
}

/**
 * @brief 功率模型
 * @todo 有待模块化,djimotor也得改改
 */
static void PowerControl() {
  // 获取电机速度反馈,化成单位rad/s
  float motor_speed_fdb[4];
  for (int i = 0; i < 4; i++) {
    motor_speed_fdb[i] = (float)chassis->wheel_motor[i]->measure.speed_aps / 6.f;
  }

  // 获取当前电机参考电流，统一位单位为A
  float motor_current_list[4];
  float motor_real_current_list[4];
  for (int i = 0; i < 4; i++) {
    motor_current_list[i] = (float)chassis->wheel_motor[i]->motor_controller.final_output;
    motor_real_current_list[i]=(float)chassis->wheel_motor[i]->measure.real_current;
  }

  float initial_give_power[4] = {0.0f};  // 每个电机的初始估计功率
  float initial_give_real_power[4] = {0.0f};  // 每个电机的初始估计功率
  float initial_total_power = 0.0f;      // 估计初始总功率
  power=0;


  // 计算每个电机的功率贡献
  for (int i = 0; i < 4; i++) {
    initial_give_power[i] =
        k0 + k1 * motor_current_list[i] / (16384.0f / 20.0f) + k2 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
        k3 * motor_current_list[i] / (16384.0f / 20.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
        k4 * motor_current_list[i] / (16384.0f / 20.0f) * motor_current_list[i] / (16384.0f / 20.0f) +
        k5 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f);

    // 只累加正向功率
    if (initial_give_power[i] > 0) {
      initial_total_power += initial_give_power[i];
    }
  }

  // 计算每个电机的功率贡献
  for (int i = 0; i < 4; i++) {
    initial_give_real_power[i] =
        k0 + k1 * motor_real_current_list[i] / (16384.0f / 20.0f) + k2 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
        k3 * motor_real_current_list[i] / (16384.0f / 20.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
        k4 * motor_real_current_list[i] / (16384.0f / 20.0f) * motor_real_current_list[i] / (16384.0f / 20.0f) +
        k5 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f);

    power += initial_give_real_power[i];
  }
  // 功率超限时进行动态调整
  if (initial_total_power > (float)chassis_ctrl_cmd->max_power) {
    float power_scale = (float)chassis_ctrl_cmd->max_power / initial_total_power;  // 削减功率比例
    float scaled_give_power[4];
    // 计算缩放后的功率目标
    for (int i = 0; i < 4; i++) {
      scaled_give_power[i] = initial_give_power[i] * power_scale;
      chassis->wheel_motor[i]->scaled_give_power = scaled_give_power[i];
    }

    // 重新计算每个电机的电流参考值
    for (int i = 0; i < 4; i++) {
      // 二次方程系数计算，参数
      float a = k4 / (16384.0f / 20.0f) / (16384.0f / 20.0f);
      float b = k1 / (16384.0f / 20.0f) + k3 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) / (16384.0f / 20.0f);
      float c = k2 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) +
                k5 * motor_speed_fdb[i] * (2.0f * PI / 60.0f) * motor_speed_fdb[i] * (2.0f * PI / 60.0f) -
                scaled_give_power[i] + k0;
      float discriminant = b * b - 4 * a * c;  // 判别式
      if (discriminant >= 0) {
        float sqrt_disc = sqrtf(discriminant);
        float temp1 = (-b + sqrt_disc) / (2 * a);
        float temp2 = (-b - sqrt_disc) / (2 * a);

        // 选择最接近当前电流的解
        if (motor_current_list[i] > 0) {
          motor_current_list[i] = (fabsf(temp1 - motor_current_list[i]) < fabsf(temp2 - motor_current_list[i]))
                                      ? fminf(16000.f, temp1)
                                      : fminf(16000.f, temp2);
        } else {
          motor_current_list[i] = (fabsf(temp1 - motor_current_list[i]) < fabsf(temp2 - motor_current_list[i]))
                                      ? fmaxf(-16000.f, temp1)
                                      : fmaxf(-16000.f, temp2);
        }
      } else {
        // 无解时归零
        motor_current_list[i] = 0.0f;
      }
    }
  }
  for (int i = 0; i < 4; i++) {
    chassis->wheel_motor[i]->motor_controller.final_output = (int16_t)(motor_current_list[i]);
  }
}


/**
 * @brief 预测电机功率并进行限制
 *
 */
static void LimitChassisOutput() {
  DJIMotorSetPIDRef(chassis->wheel_motor[0], vt_lf);
  DJIMotorSetPIDRef(chassis->wheel_motor[1], vt_rf);
  DJIMotorSetPIDRef(chassis->wheel_motor[2], vt_lb);
  DJIMotorSetPIDRef(chassis->wheel_motor[3], vt_rb);
   PowerControl();
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed() {
  // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
  // chassis_feedback_data.vx vy wz =
  // DJIMotor得改otherfeed
}

ChassisInstance* ChassisInit(Chassis_Init_Config_s* chassis_init_config) {
  ChassisInstance* chassis_instance = (ChassisInstance*)zmalloc(sizeof(ChassisInstance));
  referee_data = GetReferee();
  //初始化底盘外部IMU
  chassis_instance->chassis_external_imu = ExternalIMUInit(
      chassis_init_config->external_imu.can_id,
      chassis_init_config->external_imu.mst_id,
      chassis_init_config->external_imu.can_handle);
  chassis_instance->tof_sense = TOFSenseInit(&chassis_init_config->tof_sense_config);
  chassis_param = chassis_init_config->chassis_param;  // 在运行时赋值

  float half_wheel_base = chassis_param.wheel_base / 2.0f;
  float half_track_width = chassis_param.track_width / 2.0f;
  float center_gimbal_offset_x = chassis_param.center_gimbal_offset_x;
  float center_gimbal_offset_y = chassis_param.center_gimbal_offset_y;
  k0 = chassis_param.power_param.k0;
  k1 = chassis_param.power_param.k1;
  k2 = chassis_param.power_param.k2;
  k3 = chassis_param.power_param.k3;
  k4 = chassis_param.power_param.k4;
  k5 = chassis_param.power_param.k5;

  lf_radius = sqrtf((half_track_width + center_gimbal_offset_x) * (half_track_width + center_gimbal_offset_x) +
                    (half_wheel_base - center_gimbal_offset_y) * (half_wheel_base - center_gimbal_offset_y)) *
              DEGREE_2_RAD;

  rf_radius = sqrtf((half_track_width - center_gimbal_offset_x) * (half_track_width - center_gimbal_offset_x) +
                    (half_wheel_base - center_gimbal_offset_y) * (half_wheel_base - center_gimbal_offset_y)) *
              DEGREE_2_RAD;

  lb_radius = sqrtf((half_track_width + center_gimbal_offset_x) * (half_track_width + center_gimbal_offset_x) +
                    (half_wheel_base + center_gimbal_offset_y) * (half_wheel_base + center_gimbal_offset_y)) *
              DEGREE_2_RAD;

  rb_radius = sqrtf((half_track_width - center_gimbal_offset_x) * (half_track_width - center_gimbal_offset_x) +
                    (half_wheel_base + center_gimbal_offset_y) * (half_wheel_base + center_gimbal_offset_y)) *
              DEGREE_2_RAD;
  PIDInit(&follow_pid,&chassis_init_config->follow_pid);
  for (int i = 0; i < 4; i++) {
    chassis_init_config->wheel_motor_config[i].controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
    chassis_init_config->wheel_motor_config[i].controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
    chassis_init_config->wheel_motor_config[i].controller_setting_init_config.outer_loop_type = SPEED_LOOP;
    chassis_init_config->wheel_motor_config[i].controller_setting_init_config.close_loop_type = SPEED_LOOP;
    chassis_instance->wheel_motor[i] = DJIMotorInit(&chassis_init_config->wheel_motor_config[i]);
  }
  chassis_instance->leg_motor[1] = DMMotorInit(&chassis_init_config->leg_motor_config[1]);
   chassis_instance->leg_motor[0] = DMMotorInit(&chassis_init_config->leg_motor_config[0]);

  chassis = chassis_instance;
  chassis_ctrl_cmd = &chassis->chassis_ctrl_cmd;  // 在运行时初始化指针
  chassis_ctrl_cmd->power_distribute=1.0f;
  return chassis_instance;
}
/* 机器人底盘控制核心任务 */
void ChassisTask() {
  switch (chassis->super_cap->cap_msg.error_detect){
    case 0:
  switch (chassis->super_cap_mode) {
    case SAFETY_MODE:
      if (chassis_ctrl_cmd->SuperCapBoost == 1) {
        chassis->super_cap_mode = ACTIVE_MODE;
      }
      if (chassis->super_cap->cap_msg.cap_v > 18.0f) {
        chassis->super_cap_mode = PASSIVE_MODE;
      }
      else if (chassis->super_cap->cap_msg.cap_v < 14.0f) {
        chassis->super_cap_mode = CHARGING_MODE;
      }
      chassis->chassis_ctrl_cmd.max_power =referee_data->GameRobotState.chassis_power_limit;
      break;
      case CHARGING_MODE:
      if (chassis->super_cap->cap_msg.cap_v > 18.0f) {
        chassis->super_cap_mode = PASSIVE_MODE;
      }
      chassis->chassis_ctrl_cmd.max_power =
          referee_data->GameRobotState.chassis_power_limit -5;
      break;
    case PASSIVE_MODE:
      if (chassis_ctrl_cmd->SuperCapBoost == 1) {
        chassis->super_cap_mode = ACTIVE_MODE;
      }
      if (chassis->super_cap->cap_msg.cap_v < 14.0f) {
        chassis->super_cap_mode = CHARGING_MODE;
      }
      else if (chassis->super_cap->cap_msg.cap_v > 18.0f) {
        chassis->chassis_ctrl_cmd.max_power =referee_data->GameRobotState.chassis_power_limit;
      }
      else if (chassis->super_cap->cap_msg.cap_v >= 14.0f&&chassis->super_cap->cap_msg.cap_v <= 18.0f) {
        chassis->super_cap_mode = SAFETY_MODE;
      }
      break;
    case ACTIVE_MODE:
      if (chassis->super_cap->cap_msg.cap_v < 14.0f)
        chassis->super_cap_mode = CHARGING_MODE;
      if (chassis_ctrl_cmd->SuperCapBoost != 1)
        chassis->super_cap_mode = PASSIVE_MODE;
      chassis->chassis_ctrl_cmd.max_power = 180;
      break;
    default:
      chassis->super_cap_mode = SAFETY_MODE;
  }
    break;
      default:
      chassis_ctrl_cmd->max_power = referee_data->GameRobotState.chassis_power_limit;
      break;
}
  if (referee_data->GameRobotState.chassis_power_limit==0) {
    chassis_ctrl_cmd->max_power=100;
  }
  if (chassis->super_cap->cap_msg.cap_v==0) {
    chassis_ctrl_cmd->max_power=100;
  }

  if (chassis_ctrl_cmd->chassis_mode == CHASSIS_POWER_OFF) {
    // 如果出现重要模块离线或遥控器设置为急停,让电机停止
    for (int i = 0; i < 4; i++) DJIMotorStop(chassis->wheel_motor[i]);
    DMMotorStop(chassis->leg_motor[0]);
    DMMotorStop(chassis->leg_motor[1]);
  } else {
    // 正常工作
    for (int i = 0; i < 4; i++) DJIMotorEnable(chassis->wheel_motor[i]);
    DMMotorEnable(chassis->leg_motor[0]);
    DMMotorEnable(chassis->leg_motor[1]);
  }
  // 根据控制模式设定旋转速度
  switch (chassis_ctrl_cmd->chassis_mode)
  {
    case CHASSIS_FOLLOW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
      normal_follow_tick=DWT_GetTimeline_ms();
      follow_angle=0.f ;//跟随正前方
      //跳变处理
      if (follow_angle-chassis_ctrl_cmd->offset_angle>=180.f) {
        follow_angle-=360.0f;
      }
      else if(follow_angle-chassis_ctrl_cmd->offset_angle<=-180.f) {
        follow_angle+=360.0f;
      }
       //如果状态是刚从follow_rear_ecd切换过来，那么等待500ms再算follow_pid，其他模式切过来直接算就行，todo可以将500写入config
      if (DWT_GetTimeline_ms()-reverse_follow_tick>=800) {
        chassis_ctrl_cmd->wz+=PIDCalculate(&follow_pid,chassis_ctrl_cmd->offset_angle,follow_angle);
      }
      break;
    case CHASSIS_FOLLOW_REAR_END: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
      reverse_follow_tick=DWT_GetTimeline_ms();
      follow_angle=180.f ;//跟随车辆后方
      //跳变处理
      if (follow_angle-chassis_ctrl_cmd->offset_angle>=180.f) {
        follow_angle-=360.0f;
      }
      else if(follow_angle-chassis_ctrl_cmd->offset_angle<=-180.f) {
        follow_angle+=360.0f;
      }
      //如果状态是刚从follow切换过来，那么等待500ms再算follow_pid，其他模式切过来直接算就行，todo可以将500写入config
      if (DWT_GetTimeline_ms()-normal_follow_tick>=800) {
        chassis_ctrl_cmd->wz+=PIDCalculate(&follow_pid,chassis_ctrl_cmd->offset_angle,follow_angle);
      }
      break;
    case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
      // chassis_cmd_recv.wz = 4000;
      break;
    default:
      break;
  }
  // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
  // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系(x指向正北时y在正东)
  static float sin_theta, cos_theta;
  cos_theta = arm_cos_f32(chassis_ctrl_cmd->offset_angle * DEGREE_2_RAD);
  sin_theta = arm_sin_f32(chassis_ctrl_cmd->offset_angle * DEGREE_2_RAD);
  chassis_vx = chassis_ctrl_cmd->vx * cos_theta +chassis_ctrl_cmd->vy * sin_theta;
  chassis_vy = -chassis_ctrl_cmd->vx * sin_theta + chassis_ctrl_cmd->vy * cos_theta;
  // 根据电机的反馈速度和IMU(如果有)计算真实速度
  EstimateSpeed();

  // 根据控制模式进行正运动学解算,计算底盘输出
  MecanumCalculate();

  // 功率控制与输出限幅
  LimitChassisOutput();


  //兔腿控制
  //LegControl();
  NewLegControl();
}