#include "arm_traj.h"

#include "arm_pose_table.h"

/* 状态机。数值显式给出，便于在调试器里直接看。 */
typedef enum {
  ST_IDLE = 0,      /* 未启动 / 已结束 */
  ST_MOVE = 1,      /* PD 驱动到目标，直到到位或超时 */
  ST_SETTLE = 2,    /* 到位后等真静止 */
  ST_AVERAGE = 3,   /* 静止窗采样 */
  ST_DONE = 4,      /* 全部完成 */
} ArmTrajState;

static ArmTraj_ReadFn s_read;
static ArmTraj_OutputFn s_out;
static ArmTraj_SampleFn s_sample;

static ArmTrajState s_state = ST_IDLE;
static uint8_t  s_pose;        /* 当前位姿下标 0..ARM_POSE_COUNT-1 */
static int8_t   s_dir;         /* +1 / -1：第几遍 */
static float    s_t_state;     /* 当前状态已持续时间（秒），每次转移清零 */
static float    s_t_still;     /* 已连续静止时间（秒） */
static float    s_t_avg;       /* AVERAGE 已进行时间 */
static uint16_t s_skipped;     /* 超时跳过的停留次数 */
static bool     s_aborted;
static bool     s_approaching = true;
static float    s_move_bias = 0.0f;

/* ---- 坐标系换算与关节角限位（见 arm_traj.h 的 ArmTraj_SetJointTransform）----
 * 契约角 = (total_angle - zero) * sign，位姿表和限位宏都在契约系里。
 * 恒等换算 + 不启用限位 = 只用于单元测试的退化模式。 */
static float    s_j_zero[2] = {0.0f, 0.0f};
static float    s_j_sign[2] = {1.0f, 1.0f};
static float    s_lim[4];            /* q1_min, q1_max, q2_min, q2_max */
static bool     s_lim_en;
static bool     s_lim_tripped;
static float    s_trip_margin;       /* 越界判定余量（rad），由上层传入 */

/* 静止窗本地缓冲。
 *
 * 为什么必须缓冲：AVERAGE 可能在跑完之前被打断（机械臂重新动起来）。
 * 如果那些中断窗口的样本已经发出去了，上位机就会收到长短不一、而且
 * 短的那批没静止干净的批次 —— 粘滞项 b*dq 就漏进数据里，而且看不出异常。
 * 所以：**整窗采样先落在本地，只有干净跑满 0.4 s 才整批提交**；
 * 被中断就丢弃，窗口重来。这样上位机收到的每一段都是完整的静止窗。
 *
 * 容量 = AVERAGE 窗周期数（0.4 s @500 Hz = 200）+ 余量。 */
#define ARM_TRAJ_AVG_CAPACITY 240u

typedef struct {
  float   q1, q2, dq1, dq2, tau1, tau2;
} ArmTrajSampleRec;

static ArmTrajSampleRec s_win[ARM_TRAJ_AVG_CAPACITY];
static uint16_t s_win_n;
static uint8_t  s_win_pose;
static int8_t   s_win_dir;

/* 到位判据：角度误差足够小。用 3 度，比 PD 的稳态误差宽松，
 * 真正的"可以开始采样"由 ST_SETTLE 的静止判据决定。 */
#define ARM_TRAJ_ARRIVE_RAD  0.0523599f   /* 3 deg */
#define ARM_TRAJ_MOVE_TIMEOUT_S 5.0f      /* 移动阶段超时（含一次最长 move） */

/* 目标位姿：契约角（0 = 水平，+ 抬起），与上位机/仿真同一份表。 */
static float pose_q1(void) { return ARM_POSE_TABLE[s_pose].q1; }
static float pose_q2(void) { return ARM_POSE_TABLE[s_pose].q2; }

static void enter_state(ArmTrajState st) {
  s_state = st;
  s_t_state = 0.0f;
  s_t_still = 0.0f;
  s_t_avg = 0.0f;
  s_win_n = 0u;      /* 任何状态转移都丢弃未提交的窗口 */
  if (st == ST_MOVE) s_approaching = true;   /* 每次移动重新决定逼近方向 */
}

/* 窗口开满：整批提交给采样回调。 */
static void commit_window(void) {
  if (!s_sample || s_win_n == 0u) return;
  for (uint16_t i = 0; i < s_win_n; ++i) {
    const ArmTrajSampleRec *r = &s_win[i];
    s_sample(r->q1, r->q2, r->dq1, r->dq2, r->tau1, r->tau2,
             s_win_pose, s_win_dir);
  }
  s_win_n = 0u;
}

void ArmTraj_SetMoveBias(float bias_rad) { s_move_bias = bias_rad; }

void ArmTraj_Init(ArmTraj_ReadFn read_fn, ArmTraj_OutputFn out_fn,
                  ArmTraj_SampleFn sample_fn) {
  s_read = read_fn;
  s_out = out_fn;
  s_sample = sample_fn;
  s_pose = 0u;
  s_dir = 1;
  s_skipped = 0u;
  s_aborted = false;
  s_win_n = 0u;
  s_approaching = true;
  enter_state(ST_MOVE);
}

void ArmTraj_Abort(void) { s_aborted = true; }

bool ArmTraj_Aborted(void) { return s_aborted; }

void ArmTraj_Progress(uint8_t *pose_id, int8_t *dir, uint8_t *state) {
  if (pose_id) *pose_id = s_pose;
  if (dir) *dir = s_dir;
  if (state) *state = (uint8_t)s_state;
}

uint16_t ArmTraj_Skipped(void) { return s_skipped; }

/* 进入下一个位姿：第 1 遍跑完（dir=+1）后换方向再来一遍，然后结束。 */
static void advance(void) {
  if (s_pose + 1u < ARM_POSE_COUNT) {
    s_pose++;
    enter_state(ST_MOVE);
    return;
  }
  if (s_dir > 0) {
    s_dir = -1;
    s_pose = 0u;
    enter_state(ST_MOVE);
    return;
  }
  enter_state(ST_DONE);
}

static bool all_still(float dq1, float dq2) {
  float m1 = (dq1 < 0.0f) ? -dq1 : dq1;
  float m2 = (dq2 < 0.0f) ? -dq2 : dq2;
  return (m1 < ARM_TRAJ_STILL_RADPS) && (m2 < ARM_TRAJ_STILL_RADPS);
}

/* PD 力矩。
 *
 * 逼近方向的实现（这里是整个双向平均能不能成立的关键）：
 *
 *   s_approaching = true  时，参考被放在目标的**外侧**：
 *       r = pose + dir * bias      (dir = +1 从正侧、-1 从负侧)
 *   机械臂因此朝目标方向运动。一旦进入 |e| < APPROACH_SNAP，把参考切回
 *   **真实目标**，机械臂在目标附近停下。
 *
 * 为什么不能直接把参考留在 pose + dir*bias（我最初就是这么写的，错的）：
 *   那样机械臂停在 pose+dir*bias 而不是 pose，两遍都从 pose 逼近到被挪后的
 *   参考，**方向相同**，静摩擦符号不翻转；而且把参考挪开的那个量
 *   （bias*KP）本身成了建模误差，还会被双向平均保留下来。
 *   实测：那种写法两遍初始力矩只差 0.047 N*m，而正确写法应有 bias*KP 量级。
 */
/* 切回真实目标的半径。
 *
 * 这里踩过两次坑，都记录在此：
 *  1) SNAP 必须**明显大于** ARM_TRAJ_ARRIVE_RAD（到位判据）。最初取 4 deg、
 *     到位判据 3 deg，两者只差 1 deg：撤掉 bias 时臂还在移动，而 bias 造成的
 *     参考差是 bias*KP = 0.105*25 = 2.6 N*m —— 比很多位姿的重力矩还大，
 *     于是冲过头、来回震荡，状态机在 MOVE/SETTLE 之间反复横跳，
 *     实测每 2 万周期才前进一个位姿。
 *  2) bias 不能太大。它只需要达到"机械臂能从这一侧逼近"的程度，
 *     不需要把参考挪出去很远。2 deg 足够打断库仑摩擦，且撤掉时扰动很小。
 */
#define ARM_TRAJ_APPROACH_SNAP 0.174533f    /* 10 deg */
#define ARM_TRAJ_APPROACH_BIAS 0.0349066f   /* 2 deg */

static void pd(float q1, float q2, float dq1, float dq2,
               bool *approaching, float *t1, float *t2) {
  float e1 = pose_q1() - q1;
  float e2 = pose_q2() - q2;
  float r1 = pose_q1();
  if (*approaching) {
    float m1 = (e1 < 0.0f) ? -e1 : e1;
    float m2 = (e2 < 0.0f) ? -e2 : e2;
    if (m1 < ARM_TRAJ_APPROACH_SNAP && m2 < ARM_TRAJ_APPROACH_SNAP) {
      *approaching = false;            /* 到位附近：撤掉偏置，停在真实目标 */
    } else {
      float b = (s_move_bias > 0.0f) ? s_move_bias : ARM_TRAJ_APPROACH_BIAS;
      r1 = pose_q1() + (float)s_dir * b;
    }
  }
  *t1 = ARM_TRAJ_KP * (r1 - q1) - ARM_TRAJ_KD * dq1;
  *t2 = ARM_TRAJ_KP * (pose_q2() - q2) - ARM_TRAJ_KD * dq2;

  /* 契约系力矩 -> 电机系力矩。
   *
   * 上面的 q1/q2/dq 已经是契约角，所以算出来的力矩是**契约系**的；下发给
   * 电机必须是**电机系**。q_sign = -1 时两个系反向，不反号就是朝反方向推 ——
   * 臂一路顶到限位、永远进不了静止窗，而代码看起来完全正常。
   * gravity_comp.c 的 GravityComp_Calc 末尾做的就是同一件事，这里必须对齐。
   *
   * 这个错误是 tests/test_arm_traj_limits.py 里"sign 反转后 PD 是否还朝目标推"
   * 那条测试抓出来的：恒等配置（q_sign=+1）下完全看不出来。 */
  if (s_j_sign[0] < 0.0f) *t1 = -*t1;
  if (s_j_sign[1] < 0.0f) *t2 = -*t2;
}

void ArmTraj_SetJointTransform(float q1_zero, float q2_zero,
                               float q1_sign, float q2_sign,
                               float q1_min, float q1_max,
                               float q2_min, float q2_max,
                               float trip_margin) {
  s_j_zero[0] = q1_zero;
  s_j_zero[1] = q2_zero;
  /* 符号归一到 ±1：上层传别的一律按正负号处理，避免 0 或 2 这种值静默改变量纲 */
  s_j_sign[0] = (q1_sign < 0.0f) ? -1.0f : 1.0f;
  s_j_sign[1] = (q2_sign < 0.0f) ? -1.0f : 1.0f;
  s_lim[0] = q1_min;
  s_lim[1] = q1_max;
  s_lim[2] = q2_min;
  s_lim[3] = q2_max;
  /* 限位配置反了就整体禁用，而不是留一个"必然立刻 abort"的坏配置 */
  s_lim_en = (q1_min < q1_max) && (q2_min < q2_max);
  s_trip_margin = (trip_margin > 0.0f) ? trip_margin : 0.0f;
  s_lim_tripped = false;
}

bool ArmTraj_LimitTripped(void) { return s_lim_tripped; }

bool ArmTraj_Tick(float dt) {
  if (s_aborted) {
    if (s_out) s_out(0.0f, 0.0f);
    return false;
  }
  if (s_state == ST_IDLE) {
    if (s_out) s_out(0.0f, 0.0f);
    return false;
  }
  if (s_state == ST_DONE) {
    /* 已完成：卸力。这里返回 true 是**幂等**的，调用方每周期轮询都能看到，
     * 不会因为错过某一次轮询而永远读不到完成标志。 */
    if (s_out) s_out(0.0f, 0.0f);
    return true;
  }

  float q1_raw = 0.0f, q2_raw = 0.0f, dq1 = 0.0f, dq2 = 0.0f;
  if (!s_read) {
    if (s_out) s_out(0.0f, 0.0f);
    return false;
  }
  s_read(&q1_raw, &q2_raw, &dq1, &dq2);

  /* 电机原始系 -> 契约系。位姿表在契约系里，所以 PD 的两边必须同系；
   * 限位宏也是契约角。零位不减会在恒等配置下看不出问题，但一旦标定出
   * 真实零位，PD 就会追一个偏移了的靶子。 */
  const float q1 = (q1_raw - s_j_zero[0]) * s_j_sign[0];
  const float q2 = (q2_raw - s_j_zero[1]) * s_j_sign[1];
  /* dq 是角速度：只乘符号，不减零位 */
  dq1 *= s_j_sign[0];
  dq2 *= s_j_sign[1];

  /* ---- 关节角限位 ----
   * 超限即卸力并中止，不做"软夹紧到边界"：夹紧会让臂贴着限位长期顶着额定
   * 力矩（J1 是 28 N*m），既烧电机也伤机械；卸力才是安全态。 */
  if (s_lim_en) {
    const float m = s_trip_margin;
    if (q1 < s_lim[0] - m || q1 > s_lim[1] + m ||
        q2 < s_lim[2] - m || q2 > s_lim[3] + m) {
      s_lim_tripped = true;
      s_aborted = true;
      if (s_out) s_out(0.0f, 0.0f);
      return false;
    }
  }

  float tau1, tau2;
  pd(q1, q2, dq1, dq2, &s_approaching, &tau1, &tau2);

  s_t_state += dt;

  switch (s_state) {
    case ST_MOVE: {
      /* 到位=角度误差小且基本静止；再给一个超时兜底，防止位姿不可达时卡死。
       * 注意超时只跳过"这个位姿"，不影响整体流程。 */
      float e1 = pose_q1() - q1;
      float e2 = pose_q2() - q2;
      float m1 = (e1 < 0.0f) ? -e1 : e1;
      float m2 = (e2 < 0.0f) ? -e2 : e2;
      if ((m1 < ARM_TRAJ_ARRIVE_RAD && m2 < ARM_TRAJ_ARRIVE_RAD
           && all_still(dq1, dq2))
          || s_t_state >= ARM_TRAJ_MOVE_TIMEOUT_S) {
        enter_state(ST_SETTLE);
      }
      break;
    }

    case ST_SETTLE:
      if (all_still(dq1, dq2)) {
        s_t_still += dt;
        if (s_t_still >= (float)ARM_TRAJ_STILL_MS * 0.001f) {
          enter_state(ST_AVERAGE);
          /* 记下本窗口属于哪个位姿/方向：提交时用窗口自己的标签，
           * 而不是"提交那一刻"的 s_pose —— 后者在打断重来时会错位。 */
          s_win_pose = ARM_POSE_TABLE[s_pose].pose_id;
          s_win_dir = s_dir;
          /* 本周期不采样，从下一周期开始，保证采样都发生在静止成立之后 */
          if (s_out) s_out(tau1, tau2);
          return false;
        }
      } else {
        s_t_still = 0.0f;
      }
      if (s_t_state >= ARM_TRAJ_SETTLE_TIMEOUT_MS * 0.001f) {
        /* 没能静止：跳过该次停留，继续。缺了方向的数据会被上位机的
         * 双向配对剔除，不会污染辨识。 */
        s_skipped++;
        advance();
      }
      break;

    case ST_AVERAGE:
      /* 关键：静止不是一次性事件。进入 AVERAGE 后机械臂仍可能因为残差
       * （bias 造成的力矩差）缓慢漂动，一旦重新动起来，窗内样本就被粘滞项
       * b*dq 污染了。所以这里**持续复查**静止：动起来就退回等待，
       * 窗口重来。宁可多花时间，也不要脏样本 —— 这是整个辨识里唯一
       * "移动段必须丢弃"规则的执行点。 */
      if (!all_still(dq1, dq2)) {
        enter_state(ST_SETTLE);   /* enter_state 会丢弃未提交窗口 */
        break;
      }
      s_t_avg += dt;
      if (s_win_n < ARM_TRAJ_AVG_CAPACITY) {
        ArmTrajSampleRec *r = &s_win[s_win_n++];
        /* 存**原始**值：采样回调的入口契约是电机原始系，换算由
         * ArmIdent_Feed() 做一次（报文里也带 zero/sign 供上位机核对）。
         * 这里若存契约角，换算会被做两次，零位一非零就错。 */
        r->q1 = q1_raw; r->q2 = q2_raw; r->dq1 = dq1; r->dq2 = dq2;
        r->tau1 = tau1; r->tau2 = tau2;
      }
      if (s_t_avg >= ARM_TRAJ_AVG_S) {
        commit_window();          /* 只有跑满的窗口才提交 */
        advance();
      }
      break;

    default:
      break;
  }

  if (s_out) s_out(tau1, tau2);
  return false;
}
