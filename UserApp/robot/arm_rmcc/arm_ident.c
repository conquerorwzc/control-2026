#include "arm_ident.h"
#include "ident_proto.h"
#include "bsp_dwt.h"

/* 环形缓冲：32 帧 × 40 B = 1280 B，静态分配（不碰 zmalloc）。
 * 100 Hz 采集、100 Hz 发送时，即使串口连续堵 320 ms 也不丢数据。 */
#define ARM_IDENT_RING_FRAMES 32u

static uint8_t  s_ring[ARM_IDENT_RING_FRAMES][IDENT_FRAME_BYTES];
static volatile uint16_t s_head;    /* 生产者：ArmIdent_Feed */
static volatile uint16_t s_tail;    /* 消费者：ArmIdent_Poll */
static ArmIdent_TxReadyFn s_tx_ready;   /* 可选的发送就绪谓词，见 ArmIdent_Poll */

static USARTInstance *s_usart;
static float    s_q_zero[2];
static float    s_q_sign[2];
static uint16_t s_decim;
static uint16_t s_cnt;
static uint8_t  s_seq;
static bool     s_enabled;
static uint32_t s_ring_dropped;
static uint32_t s_sent;

void ArmIdent_Init(USARTInstance *usart, float q1_sign, float q2_sign) {
  s_usart = usart;
  s_q_zero[0] = s_q_zero[1] = 0.0f;   /* 零位不参与补偿，默认与报文一致 */
  s_q_sign[0] = (q1_sign < 0.0f) ? -1.0f : 1.0f;
  s_q_sign[1] = (q2_sign < 0.0f) ? -1.0f : 1.0f;
  s_head = s_tail = 0;
  s_decim = 5u;
  s_cnt = 0;
  s_seq = 0;
  s_enabled = true;
  s_ring_dropped = s_sent = 0;
}

void ArmIdent_SetTxReadyFn(ArmIdent_TxReadyFn fn) { s_tx_ready = fn; }

void ArmIdent_SetZero(float q1_zero, float q2_zero) {
  s_q_zero[0] = q1_zero;
  s_q_zero[1] = q2_zero;
}

void ArmIdent_SetDecim(uint16_t decim) {
  if (decim < 1u) decim = 1u;
  if (decim > 50u) decim = 50u;
  s_decim = decim;
}

void ArmIdent_Enable(bool on) { s_enabled = on; }

void ArmIdent_Stats(uint32_t *ring_dropped, uint32_t *sent) {
  if (ring_dropped) *ring_dropped = s_ring_dropped;
  if (sent) *sent = s_sent;
}

void ArmIdent_Feed(float q1_raw, float q2_raw, float dq1, float dq2,
                   float tau1_cmd, float tau2_cmd) {
  if (!s_enabled) return;
  if (++s_cnt < s_decim) return;      /* 500 Hz -> 100 Hz (decim=5) */
  s_cnt = 0;

  uint16_t next = (uint16_t)((s_head + 1u) % ARM_IDENT_RING_FRAMES);
  if (next == s_tail) {               /* 环满：丢最旧，绝不阻塞控制周期 */
    s_ring_dropped++;
    s_tail = (uint16_t)((s_tail + 1u) % ARM_IDENT_RING_FRAMES);
  }

  IdentSample s;
  /* 电机报文 -> 契约角。零位默认 0，即"报文多少就是多少"；
   * 符号保证 q 增大 = 抬起连杆（补偿律的前提）。 */
  s.q1 = (q1_raw - s_q_zero[0]) * s_q_sign[0];
  s.q2 = (q2_raw - s_q_zero[1]) * s_q_sign[1];
  /* dq 同理反向：注意 dq 是"角速度"，只乘符号，不减零位 */
  s.dq1 = dq1 * s_q_sign[0];
  s.dq2 = dq2 * s_q_sign[1];
  s.tau1_cmd = tau1_cmd;
  s.tau2_cmd = tau2_cmd;
  s.t_ms = (float)DWT_GetTimeline_ms();

  Ident_BuildData(s_ring[s_head], s_seq++, &s);
  s_head = next;
}

void ArmIdent_Poll(void) {
  if (!s_usart) return;
  if (s_tail == s_head) return;

  /* 发送前必须确认上一帧真的发完了。
   *
   * 这里曾经用一个 s_busy[] 标志，注释写着"由 TX 完成回调清掉" —— 但
   * bsp_usart 不暴露 TX 完成回调，而且本模块不能去改 bsp/module 层。
   * 结果 s_busy 只写不清，第二帧起永远发不出去（而且不报错，只是静默停住）。
   *
   * 现在改成：能不能发，由上层的就绪谓词决定。bsp 已经暴露了
   * USARTInstance.usart_handle，所以上层可以直接查 HAL 的发送状态，
   * 一行搞定，不用动任何其它层：
   *
   *     static bool uart_ready(void *h) {
   *       UART_HandleTypeDef *u = (UART_HandleTypeDef *)h;
   *       return u->gState == HAL_UART_STATE_READY &&
   *              __HAL_UART_GET_FLAG(u, UART_FLAG_TC);
   *     }
   *     ArmIdent_SetTxReadyFn(uart_ready);
   *
   * 没设置谓词时按"总是就绪"处理：靠 100 Hz 发送节奏兜底。
   * 一帧 40 字节在 115200 下占 3.47 ms，而发送间隔 10 ms，所以只要
   * 控制周期没被严重拖长就不会覆盖。真机上首次跑建议把谓词接上。 */
  if (s_tx_ready && !s_tx_ready(s_usart->usart_handle)) return;

  USARTSend(s_usart, s_ring[s_tail], IDENT_FRAME_BYTES, USART_TRANSFER_IT);
  s_sent++;
  s_tail = (uint16_t)((s_tail + 1u) % ARM_IDENT_RING_FRAMES);
}

/* -------------------------------------------------------------------------
 * 接线（写进你的 robot.c，不要新建任务）
 * ---------------------------------------------------------------------------
 *  // ---- 初始化：放在 USARTRegister + USARTServiceInit 之后 ----
 *  static USARTInstance *s_ident_usart = USARTRegister(&(USART_Init_Config_s){
 *      .usart_handle = &huart1,        // J-Link VCOM 接的那一路，8N1
 *      .recv_buff_size = 1,            // 本协议单向，不接命令
 *      .module_callback = NULL,
 *  });
 *  USARTServiceInit(s_ident_usart);
 *  ArmIdent_Init(s_ident_usart, +1.0f, -1.0f);   // 符号按实测填
 *
 *  // ---- 采集：MotorControlTask 里，算出 ff_torque 之后 ----
 *  void MotorControlTask(void) {
 *    DMMotorTask_RunAll();
 *    float ff1, ff2;
 *    GravityComp_Calc(m1->measure.total_angle, m2->measure.total_angle, &ff1, &ff2);
 *    arm->ff_torque[0] = ff1 + KP*(q1_ref - m1->measure.total_angle) - KD*m1->measure.velocity;
 *    arm->ff_torque[1] = ff2 + KP*(q2_ref - m2->measure.total_angle) - KD*m2->measure.velocity;
 *    ArmIdent_Feed(m1->measure.total_angle, m2->measure.total_angle,
 *                  m1->measure.velocity,     m2->measure.velocity,
 *                  arm->ff_torque[0],        arm->ff_torque[1]);
 *  }
 *
 *  // ---- 发送：RobotTask 循环里 ----
 *  ArmIdent_Poll();
 *
 *  // ---- 发送完成回调：清 busy 标志 ----
 *  // bsp_usart 目前没有暴露 TX 完成回调，最小改法是自己在
 *  // HAL_UART_TxCpltCallback 里按 handle 找到对应槽位清 s_busy。
 *  // 若不想改动 bsp，可以先退化成 USART_TRANSFER_DMA 并在 Poll 里
 *  // 用 __HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) 判断上一帧是否结束。
 * ------------------------------------------------------------------------- */
