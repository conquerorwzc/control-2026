

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
static float debug_footforce_l=0.0f;
static float debug_footforce_r=0.0f;
static float debug_theta=0.0f;
// 雅可比力臂监控 (Jy)
static float debug_Jy_l = 0.0f;
static float debug_Jy_r = 0.0f;
// 垂直支撑力监控 (Fy)
static float debug_Fy_total = 0.0f;
// 弹簧长度监控
static float spring_length = 0.0f;
static float spring_force = 0.0f;
static float spring_stroke = 0.0f;
// 定义常量
#define G 9.81f
#define NUM_REAR_LEGS 2.0f // 后腿数量
#define PI 3.1415926535f
#define DEG_TO_RAD  0.0174532925f  // PI / 180.0
// ==================== 机器人物理与机构参数 ====================
//  整车动态负载参数 (基于实测双秤垫高法解算)
const float M_TOTAL = 31.10f;  // 整车总质量 (kg)
const float L_F     = 0.2280f; // 重心距前轴水平距离 (m)
const float H_CG    = 0.30f;  // 重心垂直高度 (m)
const float M_LEG   = 1.8f;    // 腿重量（kg)
const float L_FRONT_TO_MOTOR = 0.40f; //前轴到后关节电机的水平距离（m）
const float M_THIGH = 0.2f;
const float L2_HALF = 0.10522f;
const float M_WHEEL = 1.8f;
//姿态解算参数
const float TRACK_WIDTH = 0.4f; // 左右腿间距 40cm
const float VMC_KP_Z = 0.0f;    // 姿态位置环 Kp
const float VMC_KD_Z = 0.00f;   // 姿态速度环 Kd (用陀螺仪数据)
const float VMC_STIFFNESS = 0.0f; // 虚拟恢复力刚度
const float TRACK_WIDTH_HALF = 0.2f;
const float WHEELBASE_FACTOR = 0.3f;
const float VMC_KP_ROLL = 0.0f, VMC_KD_ROLL = 0.00f;
const float VMC_KP_PITCH = 0.0f, VMC_KD_PITCH = 0.00f;
const float VMC_STIFF_ROLL = 00.0f, VMC_STIFF_PITCH = 00.0f;
const float CTRL_DT = 0.002f; // 2ms 周期
// ==================== 提取到外部的物理几何参数 ====================
//连杆参数
// 在全局坐标系下 (X正向朝车尾，Y正向朝地)
const float L1_X  = -0.06782f;  // D点在电机前方 -> 负值
const float L1_Y  = -0.06782f;  // D点在电机上方 -> 负值
const float L2    = 0.20944f;
const float L3    = 0.05016f;
const float L4    = 0.21472f;
const float L_EXT = 0.20680f;
const float TOP_DX =  0.02200f;
const float TOP_DY = -0.03098f;

// 【下支点】相对于 B点，X为B->C方向，Y为垂直于BC向上
const float BOT_DX = -0.05180f;
const float BOT_DY = -0.00700f;
const GasSpringPoint_t gas_spring_table[GAS_SPRING_TABLE_SIZE] = {
  {0.005f,       56.65f,     48.65f},
  {0.015f,       59.95f,     51.95f},
  {0.030f,       64.90f,     56.90f},
  {0.055f,       73.15f,     65.15f},
  {0.060f,       74.80f,     66.80f},
  {0.065f,       76.45f,     68.45f},
  {0.068f,       77.44f,     69.44f},
  {0.071f,       78.43f,     70.43f},
  {0.073f,       79.09f,     71.09f}

};
const float GAS_SPRING_FREE_LENGTH = 0.230f;
// theta=0 时，主动杆 L2 绝对水平指向车头；向下压时 theta 为正。
// 2026-05 最新实车物理标定零位 (水平为0，下压为正)
#define LEFT_MOTOR_HORIZON_OFFSET  (-2.6145f ) //
#define RIGHT_MOTOR_HORIZON_OFFSET 1.4085f  //
// 电机旋转方向系数 (如果往下压时编码器数值减小，则填 -1.0f，增大填 1.0f)
#define LEFT_MOTOR_DIR   1.0f
#define RIGHT_MOTOR_DIR  (-1.0f)
// 在文件顶部定义位置缓变速率和极限角度 (均为标准数学弧度)
#define LEG_RAMP_RATE       0.0005f  // 腿长变化速度 (弧度/ms)，数值越小动作越慢
#define UPPER_LIMIT_ANGLE   0.300f   // 上方机械限位角度 (几乎水平)
#define LOWER_LIMIT_ANGLE   1.0f   // 下方最大伸展角度 (防止顶死或奇异点)
                                   //单位都为弧度，车辆坐标系
// ==============================================================
float rad_to_deg(float encoder_val) {
  return encoder_val * (180.0f / PI);
}
#include <math.h>
/**
 * @brief  气弹簧物理长度解算 (终极向量映射版)
 */
float Kinematics_Calc_Spring_Length(float theta) {
    // =========================================================
    // 1. 正运动学解算关节坐标 (完美复刻 chassis.c)
    // =========================================================
    float B_x = -L2 * cosf(theta);
    float B_y =  L2 * sinf(theta);

    float dx = L1_X - B_x;
    float dy = L1_Y - B_y;
    float d = sqrtf(dx*dx + dy*dy);

    float gamma = atan2f(dy, dx);
    float cos_alpha = (L3*L3 + d*d - L4*L4) / (2.0f * L3 * d);
    if (cos_alpha > 1.0f) cos_alpha = 1.0f;
    if (cos_alpha < -1.0f) cos_alpha = -1.0f;
    float alpha = acosf(cos_alpha);

    float theta_3 = gamma - alpha;
    float C_x = B_x + L3 * cosf(theta_3);
    float C_y = B_y + L3 * sinf(theta_3);

    // =========================================================
    // 2. 向量法建立局部刚体坐标系
    // =========================================================

    // ------ 解析 AB 曲柄 ------
    // u_AB 为 A 指向 B 的单位方向向量
    float u_AB_x = B_x / L2;
    float u_AB_y = B_y / L2;
    // v_AB 为垂直 AB 指向上方(-Y区域)的单位法向量
    float v_AB_x = u_AB_y;
    float v_AB_y = -u_AB_x;

    // 根据局部坐标映射 P_top 全局坐标 (A点为 0,0)
    float P_top_x = 0.0f + TOP_DX * u_AB_x + TOP_DY * v_AB_x;
    float P_top_y = 0.0f + TOP_DX * u_AB_y + TOP_DY * v_AB_y;

    // ------ 解析 BC 短连杆 ------
    // u_BC 为 B 指向 C 的单位方向向量
    float vec_BC_x = C_x - B_x;
    float vec_BC_y = C_y - B_y;
    float u_BC_x = vec_BC_x / L3;
    float u_BC_y = vec_BC_y / L3;
    // v_BC 为垂直 BC 指向上方(向内侧凹折区)的单位法向量
    float v_BC_x = u_BC_y;
    float v_BC_y = -u_BC_x;

    // 根据局部坐标映射 P_bot 全局坐标 (起点是 B_x, B_y)
    float P_bot_x = B_x + BOT_DX * u_BC_x + BOT_DY * v_BC_x;
    float P_bot_y = B_y + BOT_DX * u_BC_y + BOT_DY * v_BC_y;

    // =========================================================
    // 3. 计算最终长度
    // =========================================================
    float diff_x = P_top_x - P_bot_x;
    float diff_y = P_top_y - P_bot_y;
    float length = sqrtf(diff_x*diff_x + diff_y*diff_y);

    spring_length = length;
    return length;
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
 * @brief  计算气弹簧完整解耦补偿力 (包含非线性迟滞 LUT 与粘性阻尼)
 * @param  stroke_m 气弹簧当前压缩量 (m)
 * @param  vel_m_s  气弹簧当前压缩速度 (m/s) -> 正代表正在被压缩
 * @retval 气弹簧向外推的总力 (N)
 */
float Calc_Gas_Spring_Total_Force(float stroke_m, float vel_m_s) {
    // =========================================================
    // 第一步：速度死区与方向权重混合 (解决 0 速高频震荡)
    // =========================================================
    const float VEL_DEADBAND = 0.02f; // 2cm/s 内为过渡死区
    float dir_blend = 0.0f;

    if (vel_m_s > VEL_DEADBAND) {
        dir_blend = 1.0f;  // 明确在压缩
    } else if (vel_m_s < -VEL_DEADBAND) {
        dir_blend = -1.0f; // 明确在伸展
    } else {
        dir_blend = vel_m_s / VEL_DEADBAND; // 死区内线性过渡
    }

    // 将 -1~1 的方向映射为 0~1 的权重比例 (0 对应纯 Lift，1 对应纯 Push)
    float weight_push = (dir_blend + 1.0f) / 2.0f;

    // =========================================================
    // 第二步：LUT 查表与双轨线性插值
    // =========================================================
    float push_force = gas_spring_table[0].push_N;
    float lift_force = gas_spring_table[0].lift_N;

    // 1. 安全钳位保护 (越界直接取端点值)
    if (stroke_m <= gas_spring_table[0].stroke_m) {
        push_force = gas_spring_table[0].push_N;
        lift_force = gas_spring_table[0].lift_N;
    }
    else if (stroke_m >= gas_spring_table[GAS_SPRING_TABLE_SIZE - 1].stroke_m) {
        push_force = gas_spring_table[GAS_SPRING_TABLE_SIZE - 1].push_N;
        lift_force = gas_spring_table[GAS_SPRING_TABLE_SIZE - 1].lift_N;
    }
    else {
        // 2. 遍历查找所在区间并插值
        for (uint8_t i = 0; i < GAS_SPRING_TABLE_SIZE - 1; i++) {
            float x0 = gas_spring_table[i].stroke_m;
            float x1 = gas_spring_table[i + 1].stroke_m;

            if (stroke_m >= x0 && stroke_m <= x1) {
                float ratio = (stroke_m - x0) / (x1 - x0);
                push_force = gas_spring_table[i].push_N + ratio * (gas_spring_table[i + 1].push_N - gas_spring_table[i].push_N);
                lift_force = gas_spring_table[i].lift_N + ratio * (gas_spring_table[i + 1].lift_N - gas_spring_table[i].lift_N);
                break;
            }
        }
    }

    // =========================================================
    // 第三步：静力融合与动态阻尼叠加
    // =========================================================
    // 融合得到准静态查表力 F_LUT(x, dir)
    float F_LUT = lift_force + weight_push * (push_force - lift_force);

    // 假设粘性摩擦系数 C_v = 50.0 N/(m/s) (该参数需实车调参)
    const float C_v = 50.0f;
    float F_viscous = C_v * vel_m_s;

    // 最终输出推力
    spring_force=F_LUT + F_viscous;
    return F_LUT + F_viscous;
}
/**
 * @brief 动态触地干扰观测器 (DOB) - 带气弹簧补偿版
 */
static float Estimate_Contact_Force_Math(float t_real_math, float theta_math, float vel_math, float J_y, float T_gas_assist, float *t_ext_filtered) {

  // 1. 重力期望 (悬空时需向上抬，所以带负号)
  float t_thigh = M_THIGH * G * L2_HALF * arm_cos_f32(theta_math);
  float t_wheel = M_WHEEL * G * J_y;

  // 🚨 核心修正：加入气弹簧预期
  // 气弹簧 (T_gas_assist) 把腿往下踹 (相当于正向外力)，电机必须额外出一个负向的力矩来拉住它！
  float t_expected_air = -(t_thigh + t_wheel) - T_gas_assist;

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

  // 4. 折算输出 (正值代表地面把轮子往上托)
  float safe_Jy = (J_y < 0.02f) ? 0.02f : J_y;
  float foot_force = (*t_ext_filtered) / safe_Jy;

  return foot_force;
}
static void NewLegControl() {
  static LegState_e left_leg_state = LEG_STATE_AIR_LOCK;
  static LegState_e right_leg_state = LEG_STATE_AIR_LOCK;
  static float p_des_math_l = UPPER_LIMIT_ANGLE;
  static float p_des_math_r = UPPER_LIMIT_ANGLE;
  static float base_p_des_l = UPPER_LIMIT_ANGLE;
  static float base_p_des_r = UPPER_LIMIT_ANGLE;
  static float last_base_p_des_l = UPPER_LIMIT_ANGLE;
  static float last_base_p_des_r = UPPER_LIMIT_ANGLE;
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
        pitch_angle = (-1.0f * chassis->chassis_external_imu->roll) * DEG_TO_RAD;
    }
    //读取数据用于vmc解算
    float roll_err = (0.0f - chassis->chassis_external_imu->pitch)*DEG_TO_RAD;
    float roll_gyro = chassis->chassis_external_imu->gyro[0];
    float pitch_err = (0.0f - chassis->chassis_external_imu->roll)*DEG_TO_RAD; // 假设低头为正
    float pitch_gyro = chassis->chassis_external_imu->gyro[1];

    // 2. 读取电机真实的物理状态
    float raw_angle_l = chassis->leg_motor[0]->measure.total_angle;
    float raw_angle_r = chassis->leg_motor[1]->measure.total_angle;
    float theta_math_l = (raw_angle_l - LEFT_MOTOR_HORIZON_OFFSET) * LEFT_MOTOR_DIR;
    float theta_math_r = (raw_angle_r - RIGHT_MOTOR_HORIZON_OFFSET) * RIGHT_MOTOR_DIR;
    debug_theta=theta_math_l;
    float vel_l = chassis->leg_motor[0]->measure.velocity * LEFT_MOTOR_DIR;
    float vel_r = chassis->leg_motor[1]->measure.velocity * RIGHT_MOTOR_DIR;
    float t_real_math_l = chassis->leg_motor[0]->measure.torque * LEFT_MOTOR_DIR;
    float t_real_math_r = chassis->leg_motor[1]->measure.torque * RIGHT_MOTOR_DIR;
    // 清洗数据，归一极性，方便后续数学解算
    // 开机/唤醒无缝衔接
    if (is_first_run) {
      // 宏观基准线对齐
      base_p_des_l = theta_math_l;
      base_p_des_r = theta_math_r;

      // 历史微分变量对齐 (极其重要！防止第一帧速度算出无穷大)
      last_base_p_des_l = theta_math_l;
      last_base_p_des_r = theta_math_r;

      // 最终输出量对齐
      p_des_math_l = theta_math_l;
      p_des_math_r = theta_math_r;

      left_leg_state = LEG_STATE_PROBING;
      right_leg_state = LEG_STATE_PROBING;

      is_first_run = 0;
    }

    // 3. 目标轨迹平滑生成
    // switch (chassis_ctrl_cmd->leg_mode) {
    //     case LEG_MANUAL_DOWN:
    //         p_des_math_l -= LEG_RAMP_RATE; p_des_math_r -= LEG_RAMP_RATE;
    //     break;
    //     case LEG_MANUAL_UP:
    //         p_des_math_l += LEG_RAMP_RATE; p_des_math_r += LEG_RAMP_RATE;
    //     break;
    //     default:
    //     break;
    // }

    // 限幅保护
    if (p_des_math_l > LOWER_LIMIT_ANGLE) p_des_math_l = LOWER_LIMIT_ANGLE;
    if (p_des_math_l < UPPER_LIMIT_ANGLE) p_des_math_l = UPPER_LIMIT_ANGLE;
    if (p_des_math_r > LOWER_LIMIT_ANGLE) p_des_math_r = LOWER_LIMIT_ANGLE;
    if (p_des_math_r < UPPER_LIMIT_ANGLE) p_des_math_r = UPPER_LIMIT_ANGLE;

    // 4. 全局前馈力学解算
    WheelPos_t pos_l_ff = Kinematics_Calc_Wheel(theta_math_l);
    pos_l_currentx=pos_l_ff.x;
    pos_l_currenty=pos_l_ff.y;




    float l_base_real = L_FRONT_TO_MOTOR + pos_l_ff.x;
    if(l_base_real <= 0.1f) l_base_real = 0.1f;

    float m_load_rear_total = M_TOTAL * ((L_F * arm_cos_f32(pitch_angle) + H_CG * arm_sin_f32(pitch_angle)) / l_base_real);
    float m_load_single = m_load_rear_total / NUM_REAR_LEGS-2.0f;
    if (m_load_single < 0.0f) m_load_single = 0.0f;

    float force_support = m_load_single * G * 0.71f;
    float F_x_chassis = force_support * arm_sin_f32(pitch_angle);
    float F_y_chassis = force_support * arm_cos_f32(pitch_angle);
    debug_Fy_total=F_y_chassis;


    // ============================== 【左腿独立控制块】 ==============================
    // 声明独立调试变量
    static float debug_spring_l = 0.0f;

    // ---------------- [模块 A] 真实物理观测 (DOB 专属，绝对忠于传感器) ----------------
    float Jy_real_l = 0.01f;
    WheelPos_t pos_real_l = Kinematics_Calc_Wheel(theta_math_l);
    WheelPos_t pos_real_next_l = Kinematics_Calc_Wheel(theta_math_l + 0.001f);
    if (pos_real_l.is_valid && pos_real_next_l.is_valid) {
        Jy_real_l = (pos_real_next_l.y - pos_real_l.y) / 0.001f;
    }

    // 1. 计算真实的内部气弹簧推力
    float L_gas_real_l = Kinematics_Calc_Spring_Length(theta_math_l);
    float L_gas_real_next_l = Kinematics_Calc_Spring_Length(theta_math_l + 0.001f);
    float dL_dTheta_real_l = (L_gas_real_next_l - L_gas_real_l) / 0.001f;

    float stroke_real_l = GAS_SPRING_FREE_LENGTH - L_gas_real_l;
    float gas_vel_real_l = -dL_dTheta_real_l * vel_l; // 使用真实速度，感知当前真实阻力
    float F_gas_real_l = Calc_Gas_Spring_Total_Force(stroke_real_l, gas_vel_real_l);
    float T_gas_real_l = F_gas_real_l * dL_dTheta_real_l;

    // 2. 剥离内力，获取纯净触地力
    static float filter_state_l = 0.0f;
    float contact_force_l = Estimate_Contact_Force_Math(t_real_math_l, theta_math_l, vel_l, Jy_real_l, T_gas_real_l, &filter_state_l);
    debug_footforce_l = contact_force_l;


    // ---------------- [模块 B] 核心状态机与轨迹规划 ----------------
    p_des_math_l = base_p_des_l; // 初始化
    float vmc_extra_force_l = 0.0f; // 存放 VMC 虚拟推力(N)

    // 预计算 Base 雅可比供 VMC 高度映射使用 (滤除传感器噪声)
    float safe_Jy_base_l = 0.05f;
    WheelPos_t pos_base_l = Kinematics_Calc_Wheel(base_p_des_l);
    WheelPos_t pos_base_next_l = Kinematics_Calc_Wheel(base_p_des_l + 0.001f);
    if (pos_base_l.is_valid && pos_base_next_l.is_valid) {
        float temp_Jy = (pos_base_next_l.y - pos_base_l.y) / 0.001f;
        safe_Jy_base_l = (temp_Jy < 0.05f) ? 0.05f : temp_Jy;
    }

    switch (left_leg_state) {
        case LEG_STATE_AIR_LOCK:
            if (chassis_ctrl_cmd->leg_mode == LEG_MANUAL_DOWN) {
                left_leg_state = LEG_STATE_PROBING;
            }
            break;

        case LEG_STATE_PROBING:
            base_p_des_l += LEG_RAMP_RATE * 2.0f;
            p_des_math_l = base_p_des_l;

            if (contact_force_l > 15.0f) {
                left_leg_state = LEG_STATE_STANCE;
                base_p_des_l = theta_math_l; // 接触瞬间，物理对齐
            }
            break;

        case LEG_STATE_STANCE:
            // 基础检查：遥控器强行收放
            if (chassis_ctrl_cmd->leg_mode == LEG_MANUAL_DOWN) {
                base_p_des_l += LEG_RAMP_RATE;
            } else if (chassis_ctrl_cmd->leg_mode == LEG_MANUAL_UP) {
                base_p_des_l -= LEG_RAMP_RATE;
            }
            if (base_p_des_l > LOWER_LIMIT_ANGLE) base_p_des_l = LOWER_LIMIT_ANGLE;
            if (base_p_des_l < UPPER_LIMIT_ANGLE) base_p_des_l = UPPER_LIMIT_ANGLE;

            // 脱离地面检测
            if (contact_force_l < 5.0f) {
                left_leg_state = LEG_STATE_PROBING;
                break;
            }
            if (base_p_des_l <= UPPER_LIMIT_ANGLE + 0.005f) {
                left_leg_state = LEG_STATE_AIR_LOCK;
            }

            // === VMC 解算 (高度位置环 -> 角度差模) ===
            float z_roll_l = (roll_err * VMC_KP_ROLL - roll_gyro * VMC_KD_ROLL) * TRACK_WIDTH_HALF;
            float z_pitch_l = (-pitch_err * VMC_KP_PITCH + pitch_gyro * VMC_KD_PITCH) * WHEELBASE_FACTOR;
            float delta_Z_des_l = z_roll_l + z_pitch_l;

            // 用平滑的 Base 雅可比映射为角度，杜绝高频振荡
            float delta_theta_vmc_l = delta_Z_des_l / safe_Jy_base_l;
            p_des_math_l = base_p_des_l + delta_theta_vmc_l;

            // === VMC 虚拟刚度 (发力环 -> 虚拟推力) ===
            float fy_roll_l = roll_err * VMC_STIFF_ROLL;
            float fy_pitch_l = -pitch_err * VMC_STIFF_PITCH;
            vmc_extra_force_l = fy_roll_l + fy_pitch_l; // 先存下推力，等有了终极力臂再算扭矩
            break;
    }

    // 强制截断保护
    if (p_des_math_l > LOWER_LIMIT_ANGLE) p_des_math_l = LOWER_LIMIT_ANGLE;
    if (p_des_math_l < UPPER_LIMIT_ANGLE) p_des_math_l = UPPER_LIMIT_ANGLE;

    // 【极其关键】基于极其平滑的 base_p_des 差分出期望速度，这是前馈平滑的根基！
    float v_des_math_l = (base_p_des_l - last_base_p_des_l) / CTRL_DT;
    last_base_p_des_l = base_p_des_l;


    // ---------------- [模块 C] 纯净前馈力学解算 (完全基于目标姿态，提供动态爆发力) ----------------
    float target_tff_l = 0.0f, target_kp_l = 10.0f, target_kd_l = 1.0f;

    if (left_leg_state == LEG_STATE_PROBING || left_leg_state == LEG_STATE_AIR_LOCK) {
        target_kp_l = 15.0f;
        target_kd_l = 1.5f;
        target_tff_l = 0.0f;
    }
    else if (left_leg_state == LEG_STATE_STANCE) {
        target_kp_l = 200.0f;
        target_kd_l = 5.0f;

        // 1. 目标雅可比 (根据你要去的位置算力臂，保证发力不缩水)
        float Jx_des_l = 0.0f, Jy_des_l = 0.01f;
        WheelPos_t pos_des_l = Kinematics_Calc_Wheel(p_des_math_l);
        WheelPos_t pos_des_next_l = Kinematics_Calc_Wheel(p_des_math_l + 0.001f);
        if (pos_des_l.is_valid && pos_des_next_l.is_valid) {
            Jx_des_l = (pos_des_next_l.x - pos_des_l.x) / 0.001f;
            Jy_des_l = (pos_des_next_l.y - pos_des_l.y) / 0.001f;
        }

        // 2. 目标气弹簧辅助力矩 (根据你要去的速度算阻尼，彻底消除电流锯齿)
        float L_gas_des_l = Kinematics_Calc_Spring_Length(p_des_math_l);
        float L_gas_des_next_l = Kinematics_Calc_Spring_Length(p_des_math_l + 0.001f);
        float dL_dTheta_des_l = (L_gas_des_next_l - L_gas_des_l) / 0.001f;

        float stroke_des_l = GAS_SPRING_FREE_LENGTH - L_gas_des_l;
        float gas_vel_des_l = -dL_dTheta_des_l * v_des_math_l; // 🚨 使用纯净期望速度！
        float F_gas_des_l = Calc_Gas_Spring_Total_Force(stroke_des_l, gas_vel_des_l);
        float T_gas_assist_des_l = F_gas_des_l * dL_dTheta_des_l;

        // 3. 终极前馈融合
        float vmc_torque_l = vmc_extra_force_l * Jy_des_l;
        target_tff_l = (F_x_chassis * Jx_des_l) + (F_y_chassis * Jy_des_l) + vmc_torque_l ;
    }

    // ---------------- [模块 D] 低通滤波与指令下发 ----------------
    static float actual_tff_l = 0.0f, actual_kp_l = 10.0f, actual_kd_l = 1.0f;
    const float BLEND_ALPHA = 0.08f;
    actual_tff_l += BLEND_ALPHA * (target_tff_l - actual_tff_l);
    actual_kp_l  += BLEND_ALPHA * (target_kp_l  - actual_kp_l);
    actual_kd_l  += BLEND_ALPHA * (target_kd_l  - actual_kd_l);

    // 映射回电机底层极性
    float v_des_raw_l = v_des_math_l * LEFT_MOTOR_DIR;
    float p_des_raw_l = (p_des_math_l * LEFT_MOTOR_DIR) + LEFT_MOTOR_HORIZON_OFFSET;
    left_torque_feedforward = actual_tff_l * LEFT_MOTOR_DIR;

    DMMotorSetMITRef(chassis->leg_motor[0], p_des_raw_l, v_des_raw_l, actual_kp_l, actual_kd_l, left_torque_feedforward);


   // ============================== 【右腿独立控制块】 ==============================
    // 声明独立调试变量
    static float debug_spring_r = 0.0f;

    // ---------------- [模块 A] 真实物理观测 (DOB 专属，绝对忠于传感器) ----------------
    float Jy_real_r = 0.01f;
    WheelPos_t pos_real_r = Kinematics_Calc_Wheel(theta_math_r);
    WheelPos_t pos_real_next_r = Kinematics_Calc_Wheel(theta_math_r + 0.001f);
    if (pos_real_r.is_valid && pos_real_next_r.is_valid) {
        Jy_real_r = (pos_real_next_r.y - pos_real_r.y) / 0.001f;
    }

    // 1. 计算真实的内部气弹簧推力
    float L_gas_real_r = Kinematics_Calc_Spring_Length(theta_math_r);
    float L_gas_real_next_r = Kinematics_Calc_Spring_Length(theta_math_r + 0.001f);
    float dL_dTheta_real_r = (L_gas_real_next_r - L_gas_real_r) / 0.001f;

    float stroke_real_r = GAS_SPRING_FREE_LENGTH - L_gas_real_r;
    float gas_vel_real_r = -dL_dTheta_real_r * vel_r; // 使用真实速度，感知当前真实阻力
    float F_gas_real_r = Calc_Gas_Spring_Total_Force(stroke_real_r, gas_vel_real_r);
    float T_gas_real_r = F_gas_real_r * dL_dTheta_real_r;

    // 2. 剥离内力，获取纯净触地力
    static float filter_state_r = 0.0f;
    float contact_force_r = Estimate_Contact_Force_Math(t_real_math_r, theta_math_r, vel_r, Jy_real_r, T_gas_real_r, &filter_state_r);
    debug_footforce_r = contact_force_r;


    // ---------------- [模块 B] 核心状态机与轨迹规划 ----------------
    p_des_math_r = base_p_des_r; // 初始化
    float vmc_extra_force_r = 0.0f; // 存放右腿 VMC 虚拟推力(N)

    // 预计算 Base 雅可比供 VMC 高度映射使用 (滤除传感器噪声)
    float safe_Jy_base_r = 0.05f;
    WheelPos_t pos_base_r = Kinematics_Calc_Wheel(base_p_des_r);
    WheelPos_t pos_base_next_r = Kinematics_Calc_Wheel(base_p_des_r + 0.001f);
    if (pos_base_r.is_valid && pos_base_next_r.is_valid) {
        float temp_Jy = (pos_base_next_r.y - pos_base_r.y) / 0.001f;
        safe_Jy_base_r = (temp_Jy < 0.05f) ? 0.05f : temp_Jy;
    }

    switch (right_leg_state) {
        case LEG_STATE_AIR_LOCK:
            if (chassis_ctrl_cmd->leg_mode == LEG_MANUAL_DOWN) {
                right_leg_state = LEG_STATE_PROBING;
            }
            break;

        case LEG_STATE_PROBING:
            base_p_des_r += LEG_RAMP_RATE * 2.0f;
            p_des_math_r = base_p_des_r;

            if (contact_force_r > 15.0f) {
                right_leg_state = LEG_STATE_STANCE;
                base_p_des_r = theta_math_r; // 接触瞬间，物理对齐
            }
            break;

        case LEG_STATE_STANCE:
            // 基础检查：遥控器强行收放
            if (chassis_ctrl_cmd->leg_mode == LEG_MANUAL_DOWN) {
                base_p_des_r += LEG_RAMP_RATE;
            } else if (chassis_ctrl_cmd->leg_mode == LEG_MANUAL_UP) {
                base_p_des_r -= LEG_RAMP_RATE;
            }
            if (base_p_des_r > LOWER_LIMIT_ANGLE) base_p_des_r = LOWER_LIMIT_ANGLE;
            if (base_p_des_r < UPPER_LIMIT_ANGLE) base_p_des_r = UPPER_LIMIT_ANGLE;

            // 脱离地面检测
            if (contact_force_r < 5.0f) {
                right_leg_state = LEG_STATE_PROBING;
                break;
            }
            if (base_p_des_r <= UPPER_LIMIT_ANGLE + 0.005f) {
                right_leg_state = LEG_STATE_AIR_LOCK;
            }

            // === VMC 解算 (高度位置环 -> 角度差模) ===
            // 🚨 极其关键：右腿 Roll 补偿带有天然负号 (左倾时左侧伸长，右侧缩短)
            float z_roll_r = -(roll_err * VMC_KP_ROLL - roll_gyro * VMC_KD_ROLL) * TRACK_WIDTH_HALF;
            float z_pitch_r = (-pitch_err * VMC_KP_PITCH + pitch_gyro * VMC_KD_PITCH) * WHEELBASE_FACTOR; // Pitch 共模，方向与左腿一致
            float delta_Z_des_r = z_roll_r + z_pitch_r;

            // 用平滑的 Base 雅可比映射为角度，杜绝高频振荡
            float delta_theta_vmc_r = delta_Z_des_r / safe_Jy_base_r;
            p_des_math_r = base_p_des_r + delta_theta_vmc_r;

            // === VMC 虚拟刚度 (发力环 -> 虚拟推力) ===
            // 🚨 Roll 推力取反
            float fy_roll_r = -roll_err * VMC_STIFF_ROLL;
            float fy_pitch_r = -pitch_err * VMC_STIFF_PITCH;
            vmc_extra_force_r = fy_roll_r + fy_pitch_r;
            break;
    }

    // 强制截断保护
    if (p_des_math_r > LOWER_LIMIT_ANGLE) p_des_math_r = LOWER_LIMIT_ANGLE;
    if (p_des_math_r < UPPER_LIMIT_ANGLE) p_des_math_r = UPPER_LIMIT_ANGLE;

    // 【极其关键】基于极其平滑的 base_p_des 差分出期望速度
    float v_des_math_r = (base_p_des_r - last_base_p_des_r) / CTRL_DT;
    last_base_p_des_r = base_p_des_r;


    // ---------------- [模块 C] 纯净前馈力学解算 (完全基于目标姿态，提供动态爆发力) ----------------
    float target_tff_r = 0.0f, target_kp_r = 10.0f, target_kd_r = 1.0f;

    if (right_leg_state == LEG_STATE_PROBING || right_leg_state == LEG_STATE_AIR_LOCK) {
        target_kp_r = 15.0f;
        target_kd_r = 1.5f;
        target_tff_r = 0.0f;
    }
    else if (right_leg_state == LEG_STATE_STANCE) {
        target_kp_r = 200.0f;
        target_kd_r = 5.0f;

        // 1. 目标雅可比
        float Jx_des_r = 0.0f, Jy_des_r = 0.01f;
        WheelPos_t pos_des_r = Kinematics_Calc_Wheel(p_des_math_r);
        WheelPos_t pos_des_next_r = Kinematics_Calc_Wheel(p_des_math_r + 0.001f);
        if (pos_des_r.is_valid && pos_des_next_r.is_valid) {
            Jx_des_r = (pos_des_next_r.x - pos_des_r.x) / 0.001f;
            Jy_des_r = (pos_des_next_r.y - pos_des_r.y) / 0.001f;
        }

        // 2. 目标气弹簧辅助力矩 (彻底消除右腿电流锯齿)
        float L_gas_des_r = Kinematics_Calc_Spring_Length(p_des_math_r);
        float L_gas_des_next_r = Kinematics_Calc_Spring_Length(p_des_math_r + 0.001f);
        float dL_dTheta_des_r = (L_gas_des_next_r - L_gas_des_r) / 0.001f;

        float stroke_des_r = GAS_SPRING_FREE_LENGTH - L_gas_des_r;
        float gas_vel_des_r = -dL_dTheta_des_r * v_des_math_r; // 🚨 使用纯净期望速度！
        float F_gas_des_r = Calc_Gas_Spring_Total_Force(stroke_des_r, gas_vel_des_r);
        float T_gas_assist_des_r = F_gas_des_r * dL_dTheta_des_r;

        // 3. 终极前馈融合 (虚拟推力 * 目标雅可比)
        float vmc_torque_r = vmc_extra_force_r * Jy_des_r;
        target_tff_r = (F_x_chassis * Jx_des_r) + (F_y_chassis * Jy_des_r) + vmc_torque_r ;
    }

    // ---------------- [模块 D] 低通滤波与指令下发 ----------------
    static float actual_tff_r = 0.0f, actual_kp_r = 10.0f, actual_kd_r = 1.0f;
    actual_tff_r += BLEND_ALPHA * (target_tff_r - actual_tff_r);
    actual_kp_r  += BLEND_ALPHA * (target_kp_r  - actual_kp_r);
    actual_kd_r  += BLEND_ALPHA * (target_kd_r  - actual_kd_r);

    // 映射回右腿电机底层极性
    float v_des_raw_r = v_des_math_r * RIGHT_MOTOR_DIR;
    float p_des_raw_r = (p_des_math_r * RIGHT_MOTOR_DIR) + RIGHT_MOTOR_HORIZON_OFFSET;
    right_torque_feedforward = actual_tff_r * RIGHT_MOTOR_DIR;

    DMMotorSetMITRef(chassis->leg_motor[1], p_des_raw_r, v_des_raw_r, actual_kp_r, actual_kd_r, right_torque_feedforward);
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