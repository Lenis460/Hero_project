#pragma once
// ============================================================================
// kinematics —— 坐标旋转与麦轮逆解算（纯数学，不依赖任何硬件）
//
// 数据流：遥控器(云台系速度指令 + θ) → RotateGimbalToChassis → 麦轮逆解算
//        → 四轮轮端角速度 → (驱动层套 wheel_sign 方向表) → 电机
// 本模块输出的是"虚拟轮端方向"，不含电机正反号；方向修正只允许发生在
// 驱动适配层（wheel_sign[4]），禁止改动本文件的公式符号。
// ============================================================================

#include <cmath>

#include "config.h"

namespace kinematics {

// 平面速度向量：vx 前向、vy 左向 [m/s]，wz 逆时针 [rad/s]
struct Velocity {
  float vx;
  float vy;
  float wz;
};

// 云台系 G → 底盘系 C：v_C = R(θ)·v_G
//   vx_C = cosθ·vx_G − sinθ·vy_G
//   vy_C = sinθ·vx_G + cosθ·vy_G
// θ 为云台前向相对底盘前向的逆时针角 [rad]；wz 不随坐标旋转改变
Velocity RotateGimbalToChassis(const Velocity &gimbal, float theta);

// 麦轮逆解算：底盘系速度 → 四轮轮端角速度 [rad/s]，顺序 FL,FR,RL,RR
//   w[kFL] = (vx − vy − (a+b)·wz) / r
//   w[kFR] = (vx + vy + (a+b)·wz) / r
//   w[kRL] = (vx + vy − (a+b)·wz) / r
//   w[kRR] = (vx − vy + (a+b)·wz) / r
void MecanumInverse(const Velocity &chassis, float wheel_w[kWheelCount]);

// 底盘速度限幅（cfg::kMaxVx/kMaxVy/kMaxWz），限幅后返回
Velocity Clamp(const Velocity &v);

// ---------- 轮端 <-> 电机轴换算（同一减速比） ----------
// 轮端角速度 → 电机轴角速度 [rad/s]
inline float WheelToMotorOmega(float w_wheel) {
  return w_wheel * cfg::kWheelGearRatio;
}
// 电机反馈转速 rpm → 轮端角速度 [rad/s]
inline float MotorRpmToWheelOmega(float rpm) {
  return rpm * (2.0f * cfg::kPi / 60.0f) / cfg::kWheelGearRatio;
}

// 角度归一化到 (-pi, pi]（多圈跟随误差计算用）
inline float WrapPi(float a) {
  while (a > cfg::kPi) a -= 2.0f * cfg::kPi;
  while (a <= -cfg::kPi) a += 2.0f * cfg::kPi;
  return a;
}

} // namespace kinematics
