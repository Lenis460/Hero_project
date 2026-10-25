#include "kinematics.h"

namespace kinematics {

Velocity RotateGimbalToChassis(const Velocity &gimbal, float theta) {
  const float c = cosf(theta);
  const float s = sinf(theta);
  return Velocity{
      .vx = c * gimbal.vx - s * gimbal.vy,
      .vy = s * gimbal.vx + c * gimbal.vy,
      .wz = gimbal.wz,
  };
}

void MecanumInverse(const Velocity &chassis, float wheel_w[kWheelCount]) {
  const float k = cfg::kHalfWheelBase + cfg::kHalfTrackWidth; // a+b
  const float r = cfg::kWheelRadius;
  wheel_w[kFL] = (chassis.vx - chassis.vy - k * chassis.wz) / r;
  wheel_w[kFR] = (chassis.vx + chassis.vy + k * chassis.wz) / r;
  wheel_w[kRL] = (chassis.vx + chassis.vy - k * chassis.wz) / r;
  wheel_w[kRR] = (chassis.vx - chassis.vy + k * chassis.wz) / r;
}

Velocity Clamp(const Velocity &v) {
  Velocity out = v;
  if (out.vx > cfg::kMaxVx) out.vx = cfg::kMaxVx;
  if (out.vx < -cfg::kMaxVx) out.vx = -cfg::kMaxVx;
  if (out.vy > cfg::kMaxVy) out.vy = cfg::kMaxVy;
  if (out.vy < -cfg::kMaxVy) out.vy = -cfg::kMaxVy;
  if (out.wz > cfg::kMaxWz) out.wz = cfg::kMaxWz;
  if (out.wz < -cfg::kMaxWz) out.wz = -cfg::kMaxWz;
  return out;
}

} // namespace kinematics
