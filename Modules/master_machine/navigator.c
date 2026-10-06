/**
 * @file    navigator.c
 * @brief   下位机侧导航通信协议实现 (SRM 校内赛导航通信协议 V1.0.0)
 * @note    串口: USART1, 115200 8N1, 无流控; 全部小端
 *          上 -> 下: 19 字节速度指令帧(无 cmd_id)
 *          下 -> 上: 9+data_length 字节反馈帧(有 cmd_id: 0x0001 / 0x000B)
 */
#include "navigator.h"

#include "bsp_log.h"
#include "bsp_usart.h"
#include "crc_func.h"
#include "daemon.h"
#include "referee.h"

/* 模块运行时数据 */
static navigator_recv_t recv_data;
static USARTInstance *navigator_usart_instance = NULL;
static DaemonInstance *navigator_daemon_instance = NULL;

/* 发送缓冲必须是静态的: DMA 发送期间不能被改写 */
static uint8_t tx_buffer[2][BUFFER_MAX_SIZE];
static uint8_t tx_seq = 0;         /* 下->上 帧流水号, 每帧自增、自然回绕 */
static uint8_t tx_slot = 0;        /* 反馈包轮询槽位: 0 -> 0x0001, 1 -> 0x000B */
static uint32_t last_send_tick = 0;

/* ============================ 协议层: 打包 ============================ */

/**
 * @brief 按协议打包一帧 下->上 反馈帧
 * @param cmd_id   命令码, 位于偏移 5-6 (小端)
 * @param data     数据段首地址
 * @param data_len 数据段长度
 * @param buffer   输出缓冲, 长度需 >= 9 + data_len
 * @return 帧总长度 = 9 + data_len
 * @note  crc16 覆盖偏移 0 到 7+data_len-1, 必须包含 cmd_id
 */
static uint16_t NavigatorPackFrame(uint16_t cmd_id, const uint8_t *data, uint8_t data_len, uint8_t *buffer) {
  uint16_t index = 0;

  /* 帧头: sof + data_length + seq + crc8(偏移 0-3) */
  buffer[index++] = PROTOCOL_SOF;
  buffer[index++] = (uint8_t)(data_len & 0xFF);
  buffer[index++] = (uint8_t)((data_len >> 8) & 0xFF);
  buffer[index++] = tx_seq++;
  buffer[index++] = get_CRC8_check_sum(buffer, 4, PROTOCOL_CRC8_INIT);

  /* cmd_id, 小端, 偏移 5-6 */
  buffer[index++] = (uint8_t)(cmd_id & 0xFF);
  buffer[index++] = (uint8_t)((cmd_id >> 8) & 0xFF);

  /* 数据段 */
  if (data != NULL && data_len > 0) {
    memcpy(&buffer[index], data, data_len);
    index = (uint16_t)(index + data_len);
  }

  /* 帧尾 CRC16, 低字节在前, 覆盖范围包含 cmd_id 与数据段 */
  uint16_t crc16 = get_CRC16_check_sum(buffer, index, PROTOCOL_CRC16_INIT);
  buffer[index++] = (uint8_t)(crc16 & 0xFF);
  buffer[index++] = (uint8_t)((crc16 >> 8) & 0xFF);

  return index;
}

/**
 * @brief 打包并发送一帧反馈, 非阻塞
 * @return 1: 已启动发送; 0: 串口忙(本轮跳过)或发送失败
 * @note  上一帧未发完时直接跳过, 保证帧与帧之间留有空闲, 不背靠背连发
 */
static uint8_t NavigatorTransmit(uint16_t cmd_id, const uint8_t *data, uint8_t data_len, uint8_t *buffer) {
  if (navigator_usart_instance == NULL || navigator_usart_instance->usart_handle == NULL) {
    return 0;
  }

  UART_HandleTypeDef *huart = navigator_usart_instance->usart_handle;
  if (huart->gState != HAL_UART_STATE_READY) {
    return 0;
  }

  uint16_t frame_len = NavigatorPackFrame(cmd_id, data, data_len, buffer);
  return (HAL_UART_Transmit_DMA(huart, buffer, frame_len) == HAL_OK) ? 1 : 0;
}

void navigator_send(UART_HandleTypeDef *instance, referee_info_t *referee_data) {
  (void)instance; /* 串口实例在 navigator_init() 中注册, 这里直接使用注册实例 */
  if (referee_data == NULL || navigator_usart_instance == NULL) {
    return;
  }

  /* 定频发送, 与速度指令的接收解耦; 函数非阻塞, 可以放在 1kHz 的机器人任务里 */
  uint32_t now = HAL_GetTick();
  if ((uint32_t)(now - last_send_tick) < NAV_FEEDBACK_PERIOD_MS) {
    return;
  }
  last_send_tick = now;

  if (tx_slot == 0) {
    /* cmd_id = 0x0001 比赛状态, 数据段 11 字节 */
    nav_game_status_t game_status;
    game_status.game_type = referee_data->GameState.game_type;
    game_status.game_progress = referee_data->GameState.game_progress;
    game_status.stage_remain_time = referee_data->GameState.stage_remain_time;
    game_status.sync_time_stamp = referee_data->GameState.SyncTimeStamp;
    NavigatorTransmit(NAV_CMD_ID_GAME_STATUS, (const uint8_t *)&game_status, sizeof(game_status), tx_buffer[0]);
  } else {
    /* cmd_id = 0x000B 机器人状态, 数据段 13 字节 */
    nav_robot_status_t robot_status;
    memset(&robot_status, 0, sizeof(robot_status));
    robot_status.robot_id = referee_data->GameRobotState.robot_id;
    robot_status.robot_level = referee_data->GameRobotState.robot_level;
    robot_status.current_hp = referee_data->GameRobotState.current_HP;
    robot_status.maximum_hp = referee_data->GameRobotState.maximum_HP;
    robot_status.shooter_barrel_cooling_value = referee_data->GameRobotState.shooter_barrel_cooling_value;
    robot_status.shooter_barrel_heat_limit = referee_data->GameRobotState.shooter_barrel_heat_limit;
    robot_status.chassis_power_limit = referee_data->GameRobotState.chassis_power_limit;
    robot_status.power_management_gimbal_output = referee_data->GameRobotState.power_management_gimbal_output;
    robot_status.power_management_chassis_output = referee_data->GameRobotState.power_management_chassis_output;
    robot_status.power_management_shooter_output = referee_data->GameRobotState.power_management_shooter_output;
    NavigatorTransmit(NAV_CMD_ID_ROBOT_STATUS, (const uint8_t *)&robot_status, sizeof(robot_status), tx_buffer[1]);
  }

  tx_slot ^= 1;
}

/* ============================ 协议层: 解析 ============================ */

/**
 * @brief 解析一帧 上->下 速度指令帧
 * @param frame 指向帧起点(SOF), 调用者保证可读 NAV_CMD_FRAME_LEN 字节
 * @param cmd   输出: 校验通过时写入 12 字节数据段(vx/vy/wz)
 * @param seq   输出: 帧流水号, 可为 NULL
 * @return 1: 校验全部通过; 0: 该位置不是合法帧(调用者应滑动 1 字节重新同步)
 * @note   data_length 必须为 12, 长度不符的帧整帧丢弃, 即使 CRC 全对也不写入控制量
 */
static uint8_t NavigatorParseCmdFrame(const uint8_t *frame, robot_cmd_t *cmd, uint8_t *seq) {
  /* 1. 帧头 CRC8, 覆盖偏移 0-3, 初值 0xFF */
  if (get_CRC8_check_sum((unsigned char *)frame, 4, PROTOCOL_CRC8_INIT) != frame[4]) {
    return 0;
  }

  /* 2. 数据段长度必须为 12 */
  uint16_t data_length = (uint16_t)(frame[1] | (frame[2] << 8));
  if (data_length != NAV_CMD_DATA_LEN) {
    return 0;
  }

  /* 3. CRC16, 覆盖偏移 0 到 5+data_length-1, 不包含自身 */
  uint16_t crc16_calc =
      get_CRC16_check_sum((uint8_t *)frame, PROTOCOL_HEADER_LEN + NAV_CMD_DATA_LEN, PROTOCOL_CRC16_INIT);
  uint16_t crc16_recv = (uint16_t)(frame[PROTOCOL_HEADER_LEN + NAV_CMD_DATA_LEN] |
                                   (frame[PROTOCOL_HEADER_LEN + NAV_CMD_DATA_LEN + 1] << 8));
  if (crc16_calc != crc16_recv) {
    return 0;
  }

  memcpy(cmd, &frame[PROTOCOL_HEADER_LEN], NAV_CMD_DATA_LEN);
  if (seq != NULL) {
    *seq = frame[3]; /* seq 只记录, 不作为有效性条件 */
  }
  return 1;
}

/**
 * @brief 串口接收回调: 从 DMA 缓冲里按 SOF 滑动查找并解析速度指令帧
 * @note  在 USART1 的 DMA/IDLE 中断上下文被 bsp_usart 调用
 *        上位机按 100Hz 定频单帧发送, 帧间有空闲, 因此一次回调通常包含一整帧;
 *        但这里仍然用滑动窗口, 遇到 CRC 失败只丢 1 字节继续找, 不清空整个缓冲
 */
static void DecodeNavigator(void) {
  if (navigator_usart_instance == NULL) {
    return;
  }

  uint8_t *buffer = navigator_usart_instance->recv_buff;
  uint16_t buffer_size = navigator_usart_instance->recv_buff_size;
  uint16_t index = 0;
  uint8_t frame_received = 0;

  while ((uint16_t)(index + NAV_CMD_FRAME_LEN) <= buffer_size) {
    if (buffer[index] != PROTOCOL_SOF) {
      index++; /* 不是帧头, 继续找 */
      continue;
    }

    robot_cmd_t cmd;
    uint8_t seq = 0;
    if (NavigatorParseCmdFrame(&buffer[index], &cmd, &seq) == 0) {
      recv_data.error_cnt++;
      index++; /* 丢 1 字节后重新同步(滑动窗口), 不清空整个缓冲 */
      continue;
    }

    /* 校验通过: 采用最后一次有效指令, 并持续保持该速度(协议没有超时保护, 由看门狗兜底) */
    recv_data.robot_cmd = cmd;
    recv_data.seq = seq;
    recv_data.last_update_ms = HAL_GetTick();
    recv_data.rx_frame_cnt++;
    recv_data.data_valid = 1;
    DaemonReload(navigator_daemon_instance);

    frame_received = 1;
    index = (uint16_t)(index + NAV_CMD_FRAME_LEN);
  }

  /* 本轮解析出过完整帧就清空缓冲, 防止残留字节在下一次回调中被误判成帧头
   * (bsp_usart 在回调返回后同样会清空缓冲, 这里只是提前做掉) */
  if (frame_received) {
    memset(buffer, 0, buffer_size);
  }
}

uint8_t NavigatorGetCmd(robot_cmd_t *cmd) {
  if (cmd == NULL) {
    return 0;
  }

  /* 看门狗: 超过 NAV_CMD_TIMEOUT_MS 没有收到有效帧则判定上位机离线
   * data_valid 保证上电后、收到第一帧之前不会误判为有效 */
  uint8_t valid = (uint8_t)(((uint32_t)(HAL_GetTick() - recv_data.last_update_ms) <= NAV_CMD_TIMEOUT_MS) &&
                            (recv_data.data_valid != 0));
  recv_data.data_valid = valid;

  if (valid) {
    *cmd = recv_data.robot_cmd; /* 采用最后一次有效指令 */
  } else {
    memset(cmd, 0, sizeof(robot_cmd_t)); /* 超时: 速度归零 */
  }

  return valid;
}

/**
 * @brief 离线回调, 由 daemon 任务在 500ms 未收到有效帧时调用
 * @note  打开 DMA 接收后再用 DMA/IT 发送, HAL 存在 __HAL_LOCK 互锁导致再也进不了
 *        接收中断的可能(见 master_process.c 的说明), 因此离线时重启串口服务
 */
static void NavigatorOfflineCallback(void *id) {
  (void)id;
  if (navigator_usart_instance != NULL) {
    USARTServiceInit(navigator_usart_instance);
    LOGWARNING("[navigator] nav link offline, restart usart1 rx.");
  }
}

navigator_recv_t *navigator_init(UART_HandleTypeDef *usart_handle) {
  if (usart_handle == NULL) {
    return NULL;
  }
  if (navigator_usart_instance != NULL) {
    return &recv_data; /* 导航串口只允许注册一次 */
  }

  USART_Init_Config_s conf = {
      .module_callback = DecodeNavigator,
      .recv_buff_size = NAVIGATOR_RECV_SIZE,
      .usart_handle = usart_handle,
  };
  navigator_usart_instance = USARTRegister(&conf);

  Daemon_Init_Config_s daemon_conf = {
      .reload_count = NAV_DAEMON_RELOAD_COUNT,
      .init_count = NAV_DAEMON_RELOAD_COUNT,
      .callback = NavigatorOfflineCallback,
      .owner_id = navigator_usart_instance,
  };
  navigator_daemon_instance = DaemonRegister(&daemon_conf);

  NavigatorSelfTest();
  LOGINFO("[navigator] protocol V1.0.0 ready on usart1 (115200 8N1)");

  return &recv_data;
}

/* ============================ 协议自检 ============================ */

#ifdef ESC_DEBUG
#define NAV_SELFTEST_CHECK(cond, name)                            \
  do {                                                            \
    if (!(cond)) {                                                \
      ok = 0;                                                     \
      LOGERROR("[navigator] self-test FAILED: %s", name);         \
    }                                                             \
  } while (0)

void NavigatorSelfTest(void) {
  static const uint8_t kCrcSeed[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};

  /* 文档 §5.1: 上->下 三条参考帧 */
  static const uint8_t kRefVx1[NAV_CMD_FRAME_LEN] = {0xA5, 0x0C, 0x00, 0x01, 0x26, 0x00, 0x00, 0x80, 0x3F, 0x00,
                                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6B, 0xA3};
  static const uint8_t kRefStop[NAV_CMD_FRAME_LEN] = {0xA5, 0x0C, 0x00, 0x01, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x9F};
  static const uint8_t kRefVx1VyNeg1Wz3[NAV_CMD_FRAME_LEN] = {0xA5, 0x0C, 0x00, 0x01, 0x26, 0x00, 0x00, 0x80, 0x3F,
                                                              0x00, 0x00, 0x80, 0xBF, 0x00, 0x00, 0x40, 0x40, 0xD3, 0x96};
  /* 文档 §4.3: 下->上 0x0001 参考帧 */
  static const uint8_t kRefGameStatus[20] = {0xA5, 0x0B, 0x00, 0x01, 0x5C, 0x01, 0x00, 0xAA, 0x2C, 0x01,
                                             0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0xEF, 0x74};

  uint8_t ok = 1;
  uint8_t seq = 0;
  robot_cmd_t cmd;
  uint8_t frame[32];

  /* 1. / 2. CRC 自检值 */
  NAV_SELFTEST_CHECK(get_CRC8_check_sum((unsigned char *)kCrcSeed, 9, PROTOCOL_CRC8_INIT) == 0x0B, "crc8 self-check");
  NAV_SELFTEST_CHECK(get_CRC16_check_sum((uint8_t *)kCrcSeed, 9, PROTOCOL_CRC16_INIT) == 0x6F91, "crc16 self-check");

  /* 3. 逐字节解析 §5.1 参考帧 */
  memset(&cmd, 0, sizeof(cmd));
  NAV_SELFTEST_CHECK(NavigatorParseCmdFrame(kRefVx1, &cmd, &seq) == 1, "parse vx=1,vy=0,wz=0");
  NAV_SELFTEST_CHECK(cmd.speed_vector.vx == 1.0f && cmd.speed_vector.vy == 0.0f && cmd.speed_vector.wz == 0.0f,
                     "vx=1,vy=0,wz=0 values");
  NAV_SELFTEST_CHECK(seq == 0x01, "seq of reference frame");

  memset(&cmd, 0xFF, sizeof(cmd));
  NAV_SELFTEST_CHECK(NavigatorParseCmdFrame(kRefStop, &cmd, &seq) == 1, "parse stop frame");
  NAV_SELFTEST_CHECK(cmd.speed_vector.vx == 0.0f && cmd.speed_vector.vy == 0.0f && cmd.speed_vector.wz == 0.0f,
                     "stop frame values");

  NAV_SELFTEST_CHECK(NavigatorParseCmdFrame(kRefVx1VyNeg1Wz3, &cmd, &seq) == 1, "parse vx=1,vy=-1,wz=3");
  NAV_SELFTEST_CHECK(cmd.speed_vector.vx == 1.0f && cmd.speed_vector.vy == -1.0f && cmd.speed_vector.wz == 3.0f,
                     "vx=1,vy=-1,wz=3 values");

  /* 5. data_length != 12: 整帧丢弃 */
  memcpy(frame, kRefVx1, NAV_CMD_FRAME_LEN);
  frame[1] = 0x0D;
  NAV_SELFTEST_CHECK(NavigatorParseCmdFrame(frame, &cmd, &seq) == 0, "reject data_length != 12");

  /* CRC16 出错: 整帧丢弃 */
  memcpy(frame, kRefVx1, NAV_CMD_FRAME_LEN);
  frame[5] ^= 0x01;
  NAV_SELFTEST_CHECK(NavigatorParseCmdFrame(frame, &cmd, &seq) == 0, "reject crc16 error");

  /* 4. 用 §4.3 的参数组 0x0001 反馈帧, 与参考帧逐字节比对 */
  nav_game_status_t game_status;
  memset(&game_status, 0, sizeof(game_status));
  game_status.game_type = 0xA;
  game_status.game_progress = 0xA;
  game_status.stage_remain_time = 300;
  game_status.sync_time_stamp = 0x1122334455667788ULL;

  uint8_t seq_backup = tx_seq;
  tx_seq = 0x01; /* 参考帧的 seq = 1 */
  uint16_t frame_len = NavigatorPackFrame(NAV_CMD_ID_GAME_STATUS, (const uint8_t *)&game_status,
                                          sizeof(game_status), frame);
  tx_seq = seq_backup;
  NAV_SELFTEST_CHECK(frame_len == sizeof(kRefGameStatus), "0x0001 frame length");
  NAV_SELFTEST_CHECK(memcmp(frame, kRefGameStatus, sizeof(kRefGameStatus)) == 0, "0x0001 frame bytes");

  if (ok) {
    LOGINFO("[navigator] protocol self-test PASSED");
  } else {
    LOGERROR("[navigator] protocol self-test FAILED, check crc table / frame layout!");
  }
}
#else
void NavigatorSelfTest(void) {
  /* Release 构建不带日志, 自检仅在 Debug(ESC_DEBUG) 下编译 */
}
#endif
