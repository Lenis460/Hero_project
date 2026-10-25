#pragma once
// ============================================================================
// config.h —— 全局约定与参数（本工程的"宪法"）
//
// 单位约定（全工程强制 SI）：
//   长度 m | 平移速度 m/s | 角度 rad | 角速度 rad/s | 时间 s
//
// 坐标系约定（详见 task.md「固定运动学约定」）：
//   底盘系 C：+x 车头方向，+y 车体左侧，+z 向上；+wz 俯视逆时针
//   云台系 G：+xG 云台枪口方向，+yG 云台左侧
//   θ = +xG 相对 +x 的逆时针夹角
//
// 轮子编号（从车顶俯视、车头朝上）：
//   FL=0 左前  FR=1 右前  RL=2 左后  RR=3 右后
//   所有数组、CAN ID 表、方向标定表都按此顺序
//
// 规则：任何模块需要常数，只允许 #include 本文件，禁止自行定义/改写数值；
//      标注 [待确认] 的项在基地实测后只修改本文件。
// ============================================================================

#include <cstdint>

// ---------- 轮子编号 ----------
enum WheelIndex : uint8_t {
  kFL = 0,  // 左前
  kFR = 1,  // 右前
  kRL = 2,  // 左后
  kRR = 3,  // 右后
};
inline constexpr uint8_t kWheelCount = 4;

namespace cfg {

// ---------- 数学常数 ----------
constexpr float kPi = 3.14159265358979f;
constexpr float kDeg2Rad = kPi / 180.0f;
constexpr float kRad2Deg = 180.0f / kPi;

// ---------- 底盘几何（初值，[待确认] 基地实测） ----------
constexpr float kWheelRadius     = 0.07786f; // 轮半径 r [m]
constexpr float kHalfWheelBase   = 0.275f;   // a：中心→前后轮轴线 [m]（前后轴距 0.55/2）
constexpr float kHalfTrackWidth  = 0.225f;   // b：中心→左右轮线   [m]（左右轮距 0.45/2）
constexpr float kWheelGearRatio  = 14.0f;    // 轮电机减速比 [待确认]

// ---------- 安全参数 ----------
constexpr float kRcTimeout        = 0.100f; // 遥控器失联判定阈值 [s]
constexpr float kBootDelay        = 2.000f; // 上电安全等待（含零漂采集）[s]
constexpr float kBoardLinkTimeout = 0.100f; // 上下板通信失联阈值 [s]
constexpr float kSpinOmegaStart   = 0.200f; // 小陀螺首次起步角速度 [rad/s]

// ---------- 底盘速度限幅（保守初值，M3 落地后再逐步放宽） ----------
constexpr float kMaxVx = 2.0f; // [m/s]
constexpr float kMaxVy = 2.0f; // [m/s]
constexpr float kMaxWz = 3.0f; // [rad/s]

} // namespace cfg
