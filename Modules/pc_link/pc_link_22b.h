#pragma once

#include <stdint.h>

#include "bsp_usb.h"
#include "ins_task.h"

#define PC_LINK22_FRAME_LEN 22u
#define PC_LINK22_SOF 0xCDu
#define PC_LINK22_EOF 0xDCu
#define PC_LINK22_RX_CACHE_LEN 128u
#define PC_LINK22_TIMEOUT_MS 100u
#define PC_LINK22_TX_PERIOD_MS 10u

typedef struct {
  float pitch;
  float yaw;
  float pitch_rate;
  float yaw_rate;
  uint32_t timestamp_ms;
} PCLink22Frame_s;

typedef struct {
  INS_t *imu;
  uint8_t *usb_rx_buf;

  PCLink22Frame_s rx_frame;
  PCLink22Frame_s tx_frame;

  uint8_t rx_valid;
  uint32_t last_rx_tick;
  uint32_t last_tx_tick;

  uint8_t rx_cache[PC_LINK22_RX_CACHE_LEN];
  uint16_t rx_cache_len;
} PCLink22Instance;

PCLink22Instance *PCLink22Init(INS_t *imu);
void PCLink22Send(PCLink22Instance *ins);

const PCLink22Frame_s *PCLink22GetRx(PCLink22Instance *ins);
uint8_t PCLink22RxValid(PCLink22Instance *ins);
uint8_t PCLink22IsOnline(PCLink22Instance *ins);
