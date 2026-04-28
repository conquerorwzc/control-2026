#include "pc_link_22b.h"

#include <string.h>

#include "main.h"

static PCLink22Instance pc_link22_instance;
static PCLink22Instance *pc_link22_singleton = NULL;

static void PCLink22ParseFrame(PCLink22Instance *ins, const uint8_t *frame) {
  if (ins == NULL || frame == NULL) {
    return;
  }

  memcpy(&ins->rx_frame.pitch, frame + 1, 4);
  memcpy(&ins->rx_frame.yaw, frame + 5, 4);
  memcpy(&ins->rx_frame.pitch_rate, frame + 9, 4);
  memcpy(&ins->rx_frame.yaw_rate, frame + 13, 4);
  memcpy(&ins->rx_frame.timestamp_ms, frame + 17, 4);

  ins->rx_valid = 1;
  ins->last_rx_tick = HAL_GetTick();
}

static void PCLink22TryExtractFrames(PCLink22Instance *ins) {
  uint16_t sof_idx;
  uint16_t remain_len;

  if (ins == NULL) {
    return;
  }

  while (ins->rx_cache_len > 0) {
    sof_idx = 0;
    while (sof_idx < ins->rx_cache_len && ins->rx_cache[sof_idx] != PC_LINK22_SOF) {
      sof_idx++;
    }

    if (sof_idx >= ins->rx_cache_len) {
      ins->rx_cache_len = 0;
      return;
    }

    if (sof_idx > 0) {
      remain_len = ins->rx_cache_len - sof_idx;
      memmove(ins->rx_cache, ins->rx_cache + sof_idx, remain_len);
      ins->rx_cache_len = remain_len;
    }

    if (ins->rx_cache_len < PC_LINK22_FRAME_LEN) {
      return;
    }

    if (ins->rx_cache[PC_LINK22_FRAME_LEN - 1] == PC_LINK22_EOF) {
      PCLink22ParseFrame(ins, ins->rx_cache);

      remain_len = ins->rx_cache_len - PC_LINK22_FRAME_LEN;
      if (remain_len > 0) {
        memmove(ins->rx_cache, ins->rx_cache + PC_LINK22_FRAME_LEN, remain_len);
      }
      ins->rx_cache_len = remain_len;
    } else {
      remain_len = ins->rx_cache_len - 1;
      if (remain_len > 0) {
        memmove(ins->rx_cache, ins->rx_cache + 1, remain_len);
      }
      ins->rx_cache_len = remain_len;
    }
  }
}

static void PCLink22FeedBytes(PCLink22Instance *ins, const uint8_t *data, uint16_t len) {
  uint16_t free_len;
  uint16_t drop_len;

  if (ins == NULL || data == NULL || len == 0) {
    return;
  }

  if (len >= PC_LINK22_RX_CACHE_LEN) {
    data += (len - PC_LINK22_RX_CACHE_LEN);
    len = PC_LINK22_RX_CACHE_LEN;
    ins->rx_cache_len = 0;
  }

  free_len = PC_LINK22_RX_CACHE_LEN - ins->rx_cache_len;
  if (len > free_len) {
    drop_len = len - free_len;
    if (drop_len < ins->rx_cache_len) {
      memmove(ins->rx_cache, ins->rx_cache + drop_len, ins->rx_cache_len - drop_len);
      ins->rx_cache_len -= drop_len;
    } else {
      ins->rx_cache_len = 0;
    }
  }

  memcpy(ins->rx_cache + ins->rx_cache_len, data, len);
  ins->rx_cache_len += len;
  PCLink22TryExtractFrames(ins);
}

static void PCLink22RxCallback(uint16_t recv_len) {
  if (pc_link22_singleton == NULL || pc_link22_singleton->usb_rx_buf == NULL) {
    return;
  }

  PCLink22FeedBytes(pc_link22_singleton, pc_link22_singleton->usb_rx_buf, recv_len);
}

static void PCLink22PackFrame(const PCLink22Frame_s *frame, uint8_t *buf) {
  buf[0] = PC_LINK22_SOF;
  memcpy(buf + 1, &frame->pitch, 4);
  memcpy(buf + 5, &frame->yaw, 4);
  memcpy(buf + 9, &frame->pitch_rate, 4);
  memcpy(buf + 13, &frame->yaw_rate, 4);
  memcpy(buf + 17, &frame->timestamp_ms, 4);
  buf[21] = PC_LINK22_EOF;
}

PCLink22Instance *PCLink22Init(INS_t *imu) {
  USB_Init_Config_s conf = {0};

  memset(&pc_link22_instance, 0, sizeof(pc_link22_instance));
  pc_link22_instance.imu = imu;

  conf.rx_cbk = PCLink22RxCallback;
  pc_link22_instance.usb_rx_buf = USBInit(conf);

  pc_link22_singleton = &pc_link22_instance;
  return &pc_link22_instance;
}

void PCLink22Send(PCLink22Instance *ins) {
  uint8_t tx_buf[PC_LINK22_FRAME_LEN];
  uint32_t now_tick;

  if (ins == NULL || ins->imu == NULL) {
    return;
  }

  now_tick = HAL_GetTick();
  if (now_tick - ins->last_tx_tick < PC_LINK22_TX_PERIOD_MS) {
    return;
  }
  ins->last_tx_tick = now_tick;

  ins->tx_frame.pitch = ins->imu->Pitch;
  ins->tx_frame.yaw = ins->imu->Yaw;
  ins->tx_frame.pitch_rate = ins->imu->Gyro[0];
  ins->tx_frame.yaw_rate = ins->imu->Gyro[2];
  ins->tx_frame.timestamp_ms = now_tick;

  PCLink22PackFrame(&ins->tx_frame, tx_buf);
  USBTransmit(tx_buf, PC_LINK22_FRAME_LEN);
}

const PCLink22Frame_s *PCLink22GetRx(PCLink22Instance *ins) {
  if (ins == NULL) {
    return NULL;
  }
  return &ins->rx_frame;
}

uint8_t PCLink22IsOnline(PCLink22Instance *ins) {
  if (ins == NULL || ins->last_rx_tick == 0u) {
    return 0;
  }
  return (HAL_GetTick() - ins->last_rx_tick) <= PC_LINK22_TIMEOUT_MS;
}

uint8_t PCLink22RxValid(PCLink22Instance *ins) {
  if (ins == NULL) {
    return 0;
  }
  return ins->rx_valid && PCLink22IsOnline(ins);
}
