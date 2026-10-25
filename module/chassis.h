#pragma once
// ============================================================================
// Chassis —— 底盘控制器（遥控 -> 坐标旋转 -> 麦轮解算 -> 四轮速度环）
//
// 数据流（每个控制周期一次 Update）：
//   RcInput(云台系指令) + theta(云台相对底盘角)
//     -> [分离]  旋转到底盘系 -> 麦轮解算 -> 四轮目标 [rad/s 轮端]
//     -> [跟随]  同分离 + follow_pid(目标 0, 反馈 WrapPi(theta)) -> wz 修正
//     -> [小陀螺] 同分离 + wz 叠加 cfg::kSpinOmegaStart 常值自转
//     -> 每轮目标过 Clamp 限幅；模式切换时 ResetControllers() 防积分跳变
//
// 反馈接线说明：wheel_pid_ 的反馈当前恒为 0（占位），M2 接入电机驱动后
// 传入真实轮端角速度 [rad/s]（与目标同侧同单位，见 task.md 单位约定）。
// ============================================================================

#include <cstdint>

#include "mode.h" // ChassisMode 完整定义与拨杆映射
#include "pid.h"

struct RcInput { // 与 Dr16 解耦，便于离线假输入测试
  float lh, lv, rh, rv, wheel; // 均为 -1~1
};

class Chassis {
public:
  struct Output {
    float wheel_target[4]; // FL,FR,RL,RR 轮端角速度目标 [rad/s]
    float motor_cmd[4];    // 速度环 PID 输出（M2 起为电流指令，当前假输出）
    float wz_total;        // 本周期实际使用的底盘角速度 [rad/s]（调试观察）
  };

  Chassis();

  void SetMode(ChassisMode m);
  ChassisMode mode() const { return mode_; }

  // theta：云台 +xG 相对底盘 +x 的逆时针角 [rad]
  Output Update(const RcInput &rc, float theta, float dt);

  // 模式切换/进入 SAFE_STOP 时调用：清 PID 积分与微分历史
  void ResetControllers();

  // M2 起由电机反馈层调用：写入轮端角速度反馈 [rad/s]
  void SetWheelFeedback(const float w_wheel[4]);
  const Pid &wheel_pid(uint8_t i) const { return wheel_pid_[i & 3]; }

private:
  ChassisMode mode_ = ChassisMode::kSeparate;
  Pid wheel_pid_[4];
  Pid follow_pid_;
  float wheel_feedback_[4] = {0, 0, 0, 0};
};
