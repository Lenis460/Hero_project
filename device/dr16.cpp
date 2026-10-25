#include "dr16.h"

namespace remote_control {

namespace {

// 11 位原始值 -> SwitchState；DJI 协议：1=上 3=中 2=下，其余视为无效
SwitchState ToSwitchState(uint8_t s) {
  switch (s) {
  case 1: return SwitchState::kUp;
  case 2: return SwitchState::kDown;
  case 3: return SwitchState::kMiddle;
  default: return SwitchState::kUnknown;
  }
}

} // namespace

float Dr16::ChannelToNorm(uint16_t raw) {
  float v = (static_cast<float>(raw) - 1024.0f) / 660.0f; // 满偏 ±660
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  if (v > -kStickDeadzone && v < kStickDeadzone) v = 0.0f;
  return v;
}

void Dr16::decode(const uint8_t *b) {
  // ---- 位解析（DJI 官方例程布局，禁止改动） ----
  ch_raw_[0] = (b[0] | (b[1] << 8)) & 0x07FF;
  ch_raw_[1] = ((b[1] >> 3) | (b[2] << 5)) & 0x07FF;
  ch_raw_[2] = ((b[2] >> 6) | (b[3] << 2) | (b[4] << 10)) & 0x07FF;
  ch_raw_[3] = ((b[4] >> 1) | (b[5] << 7)) & 0x07FF;
  const uint8_t s_left = ((b[5] >> 4) & 0x000C) >> 2; // buf[5] bits 7:6
  const uint8_t s_right = (b[5] >> 4) & 0x0003;       // buf[5] bits 5:4
  ch_raw_[4] = (b[16] | (b[17] << 8)) & 0x07FF;       // wheel 原始值

  // ---- 通道映射到语义接口（M1 实车确认后只改 kChXX 常量） ----
  lh_ = ChannelToNorm(ch_raw_[kChLh]);
  lv_ = ChannelToNorm(ch_raw_[kChLv]);
  rh_ = ChannelToNorm(ch_raw_[kChRh]);
  rv_ = ChannelToNorm(ch_raw_[kChRv]);
  wheel_ = ChannelToNorm(ch_raw_[4]);
  l_switch_ = ToSwitchState(s_left);
  r_switch_ = ToSwitchState(s_right);

  ++frame_count_;
}

} // namespace remote_control
