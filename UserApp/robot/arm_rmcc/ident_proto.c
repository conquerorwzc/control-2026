#include "ident_proto.h"
#include <string.h>

uint16_t Ident_Crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFFu;
  for (uint16_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)((uint16_t)data[i] << 8);
    for (uint8_t b = 0; b < 8; ++b) {
      /* 显式转回 uint16_t：整型提升会把 crc<<1 抬成 int，再异或就触发
         -Wconversion 的有符号性告警。语义不变，只是把提升写明。 */
      uint16_t shifted = (uint16_t)(crc << 1);
      crc = (crc & 0x8000u) ? (uint16_t)(shifted ^ 0x1021u) : shifted;
    }
  }
  return crc;
}

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_f32(uint8_t *p, float v) { memcpy(p, &v, 4); }

static uint16_t hdr(uint8_t *out, uint8_t type, uint8_t seq, uint16_t len, float t_ms) {
  uint16_t n = 0;
  out[n++] = IDENT_SYNC0;
  out[n++] = IDENT_SYNC1;
  out[n++] = type;
  out[n++] = seq;
  put_u16(out + n, len); n += 2;
  put_f32(out + n, t_ms); n += 4;
  return n;
}

static uint16_t finish(uint8_t *out, uint16_t n) {
  put_u16(out + n, Ident_Crc16(out + 2, (uint16_t)(n - 2)));
  return (uint16_t)(n + 2u);
}

uint16_t Ident_BuildData(uint8_t *out, uint8_t seq, const IdentSample *s) {
  uint16_t n = hdr(out, IDENT_TYPE_DATA, seq, IDENT_PAYLOAD_BYTES, s->t_ms);
  put_f32(out + n, s->q1);       n += 4;
  put_f32(out + n, s->q2);       n += 4;
  put_f32(out + n, s->dq1);      n += 4;
  put_f32(out + n, s->dq2);      n += 4;
  put_f32(out + n, s->tau1_cmd); n += 4;
  put_f32(out + n, s->tau2_cmd); n += 4;
  put_f32(out + n, s->t_ms);     n += 4;
  return finish(out, n);
}

uint16_t Ident_BuildStatus(uint8_t *out, uint8_t seq, const IdentStatus *st) {
  uint16_t n = hdr(out, IDENT_TYPE_STATUS, seq, 28u, 0.0f);
  put_f32(out + n, st->q1_zero); n += 4;
  put_f32(out + n, st->q2_zero); n += 4;
  put_f32(out + n, st->q1_sign); n += 4;
  put_f32(out + n, st->q2_sign); n += 4;
  out[n++] = st->logging;
  out[n++] = (uint8_t)(st->decim & 0xFF);
  out[n++] = (uint8_t)(st->decim >> 8);
  out[n++] = (uint8_t)st->dropped;
  put_f32(out + n, (float)st->total); n += 4;
  return finish(out, n);
}
