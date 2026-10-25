#include "dm_motor.h"

#include <cstring>

namespace dm4340 {

uint8_t BuildPosVelFrame(const Config &cfg, float p_des, float v_des,
                         uint8_t *out) {
  // 软件限幅：目标位置永远钳在 [pos_min, pos_max]（不允许靠堵转找限位）
  if (p_des > cfg.pos_max) p_des = cfg.pos_max;
  if (p_des < cfg.pos_min) p_des = cfg.pos_min;
  if (v_des < 0.0f) v_des = -v_des; // v_des 是梯形限速值，恒正

  std::memcpy(out, &p_des, 4); // 小端 float32（Cortex-M 天然小端，手册：低位在前）
  std::memcpy(out + 4, &v_des, 4);
  return 8; // 发送时帧 ID = kPosVelCmdBase + cfg.can_id
}

namespace {

// 手册 p14 线性映射的逆运算（有符号定点 -> 浮点）
inline float Scaled16(int16_t raw, float range) {
  return static_cast<float>(raw) * (2.0f * range) / 65535.0f;
}
inline float Scaled12(int16_t raw, float range) {
  return static_cast<float>(raw) * (2.0f * range) / 4095.0f;
}

inline int16_t AsSigned12(uint16_t raw) { // 12 位补码 -> int16
  return static_cast<int16_t>(raw >= 0x800 ? raw - 0x1000 : raw);
}

} // namespace

void ParseFeedback(const Config &cfg, const uint8_t *d, Feedback *out) {
  // D0 = ID | ERR<<4；D1..2 POS 大端 int16；VEL 12 位 = D3<<4 | D4>>4；
  // T 12 位 = (D4&0xF)<<8 | D5；D6 T_MOS；D7 T_Rotor
  out->id = d[0] & 0x0F;
  out->err = d[0] >> 4;
  const int16_t pos_raw = static_cast<int16_t>((d[1] << 8) | d[2]);
  const int16_t vel_raw = AsSigned12((d[3] << 4) | (d[4] >> 4));
  const int16_t tq_raw = AsSigned12(((d[4] & 0x0F) << 8) | d[5]);
  out->pos = Scaled16(pos_raw, cfg.pos_range);
  out->vel = Scaled12(vel_raw, cfg.vel_range);
  out->torque = Scaled12(tq_raw, cfg.tq_range);
  out->t_mos = d[6];
  out->t_coil = d[7];
}

void BuildEnableFrame(uint8_t *out) {
  const uint8_t kFrame[8] = {0xFF, 0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  std::memcpy(out, kFrame, 8);
}

void BuildDisableFrame(uint8_t *out) {
  const uint8_t kFrame[8] = {0xFF, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  std::memcpy(out, kFrame, 8);
}

void BuildClearErrorFrame(uint8_t *out) {
  const uint8_t kFrame[8] = {0xFF, 0xFB, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  std::memcpy(out, kFrame, 8);
}

void Driver::OnFeedback(const uint8_t *data) {
  ParseFeedback(cfg_, data, &fb_);
  online_ = true;
}

} // namespace dm4340
