#pragma once
// ============================================================================
// mode —— 控制模式定义与拨杆映射（映射为暂定方案，M1 上车确认后只改本文件）
//
// 暂定方案（对应 DT7 拨杆三态 上=1 中=3 下=2）：
//   右拨杆：上 = 分离模式 | 中 = 跟随模式 | 下 = 小陀螺
//   左拨杆：下 = 急停（SAFE_STOP）| 上/中 = 正常
//   摇杆：左杆 lv=前后 [m/s]、lh=左右 [m/s]；拨轮 = 手动旋转分量 [rad/s]
// ============================================================================

#include "dr16.h"

enum class ChassisMode : uint8_t {
  kSeparate = 0, // 分离：底盘按云台系指令运动，底盘朝向不管
  kFollow,       // 跟随：叠加"底盘 x 轴对齐云台 x 轴"的旋转分量
  kSpin,         // 小陀螺：恒定自转分量 + 云台系平移
};

// 右拨杆 -> 底盘模式（无效值返回 kSeparate，偏保守：无自转）
inline ChassisMode MapChassisMode(remote_control::SwitchState s) {
  switch (s) {
  case remote_control::SwitchState::kUp: return ChassisMode::kSeparate;
  case remote_control::SwitchState::kMiddle: return ChassisMode::kFollow;
  case remote_control::SwitchState::kDown: return ChassisMode::kSpin;
  default: return ChassisMode::kSeparate;
  }
}

// 左拨杆 -> 急停（下 = 急停，暂定）
inline bool IsEstop(remote_control::SwitchState left) {
  return left == remote_control::SwitchState::kDown;
}
