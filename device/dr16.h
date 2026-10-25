#pragma once
// ============================================================================
// Dr16 —— 大疆 DT7/DR16 遥控器 DBus 帧解码（纯解析，不依赖 HAL）
//
// 帧格式（18 字节，100000bps 偶校验，DJI 官方例程位布局）：
//   ch0 = buf[0]  | buf[1]<<8                    & 0x7FF   右摇杆水平(暂定)
//   ch1 = buf[1]>>3 | buf[2]<<5                  & 0x7FF   右摇杆垂直(暂定)
//   ch2 = buf[2]>>6 | buf[3]<<2 | buf[4]<<10     & 0x7FF   左摇杆水平(暂定)
//   ch3 = buf[4]>>1 | buf[5]<<7                  & 0x7FF   左摇杆垂直(暂定)
//   s_left  = buf[5] bits 7:6（暂定左拨杆）  s_right = buf[5] bits 5:4（暂定右拨杆）
//   wheel   = (buf[16] | buf[17]<<8) & 0x7FF - 1024          侧边拨轮
//   原始通道范围 364~1684，中值 1024，满偏 ±660
//
// !!! M1 实车确认清单 !!!（task.md M1 已列）：
//   摇杆左右/上下的通道号映射、两个拨杆的 s 位映射，以实际打印为准；
//   若与上面"暂定"标注不符，只改本文件头部的通道映射宏，不动位解析。
//
// 使用方式（对照考核说明示例）：
//   main 中 HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buf, kRxBufLen) 起流；
//   HAL_UARTEx_RxEventCallback 中 Size==kRcRxDataLen 时调 decode(rx_buf)，
//   然后重新挂起 DMA。
//
// 失联检测不在本类内（保持纯计算）：上层 SafetyGate 监视 frame_count()，
// 若在 kRcTimeout 内不增长即判失联。decode() 在 UART 回调（中断上下文）
// 执行，只做赋值，不加锁不打印。
// ============================================================================

#include <cstdint>

namespace remote_control {

inline constexpr uint16_t kRcRxDataLen = 18;         // DBus 单帧长度
inline constexpr uint8_t kRxBufLen = kRcRxDataLen * 2; // DMA 缓冲长度（示例用法）

enum class SwitchState : uint8_t {
  kUnknown = 0, // 断联/无效值
  kUp = 1,
  kDown = 2,
  kMiddle = 3,
};

class Dr16 {
public:
  // 解析一帧（frame 长度必须为 kRcRxDataLen，由回调里 Size 判断保证）
  void decode(const uint8_t *frame);

  // 摇杆/拨轮，已映射到 -1~1，含死区（|v|<kStickDeadzone 视为 0）
  float rc_lh() const { return lh_; }
  float rc_lv() const { return lv_; }
  float rc_rh() const { return rh_; }
  float rc_rv() const { return rv_; }
  float rc_wheel() const { return wheel_; }

  SwitchState rc_l_switch() const { return l_switch_; }
  SwitchState rc_r_switch() const { return r_switch_; }

  // 原始 11 位通道值（0~2047，调试打印用）；4 = wheel 原始值
  uint16_t ch_raw(uint8_t i) const { return ch_raw_[i & 0x7]; }

  // 累计有效帧计数；上层用于失联判断（周期内不增长 => 失联）
  uint32_t frame_count() const { return frame_count_; }

  static constexpr float kStickDeadzone = 0.05f;

private:
  static float ChannelToNorm(uint16_t raw); // 364~1684 -> -1~1，含死区与限幅

  // 通道映射（M1 实车确认后只改这里）
  static constexpr uint8_t kChLh = 2; // 左摇杆水平
  static constexpr uint8_t kChLv = 3; // 左摇杆垂直
  static constexpr uint8_t kChRh = 0; // 右摇杆水平
  static constexpr uint8_t kChRv = 1; // 右摇杆垂直

  float lh_ = 0.0f, lv_ = 0.0f, rh_ = 0.0f, rv_ = 0.0f, wheel_ = 0.0f;
  SwitchState l_switch_ = SwitchState::kUnknown;
  SwitchState r_switch_ = SwitchState::kUnknown;
  uint16_t ch_raw_[5] = {1024, 1024, 1024, 1024, 1024};
  uint32_t frame_count_ = 0;
};

} // namespace remote_control
