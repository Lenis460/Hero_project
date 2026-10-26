#pragma once
// ============================================================================
// C620 —— M3508 无刷电机调速器 CAN 反馈解析
//
// 反馈帧（1kHz 广播，标准帧，ID = 0x200 + 电调 ID，DLC=8）：
//   D[0..1] 转子转速 int16 大端 [rpm]（减速箱前，即电机轴）
//   D[2..3] 实际转矩原始值 int16 大端（-16384~16384 ↔ -20~20 N·m，满量程待标定）
//   D[4..5] 输出轴电角度原始值 int16 大端（0~8191 ↔ 0~360°）
//   D[6]    电机温度 [℃]
//   D[7]    空
// 控制帧（M2 起使用，0x200 起）：见 bsp/config.h kC620CurrentFrameBase
// ============================================================================

#include <cstdint>

namespace c620 {

struct Feedback {
  float rotor_rpm = 0.0f;  // 转子转速 [rpm]，电机轴侧
  int16_t torque_raw = 0;  // 转矩原始值
  uint16_t angle_raw = 0;  // 输出轴电角度原始值 0~8191
  uint8_t temp_c = 0;      // 温度 [℃]
};

// 解析一帧；esc_id 为电调拨码 ID（1~8），解析结果按 cfg::kWheelCanId
// 映射到轮位后写入 fb_out[wheel_index]
// 返回 true 表示该帧属于本车四个轮电调之一
bool OnFrame(uint8_t esc_id, const uint8_t *d, Feedback fb_out[4]);

// 转子 rpm → 轮端角速度 [rad/s]（÷ 减速比，× 2π/60）
float RpmToWheelOmega(float rpm);

} // namespace c620
