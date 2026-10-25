#include "chassis.h"

#include "config.h"
#include "kinematics.h"

namespace {

// ---- 速度环 / 跟随环 PID 初值（占位，M2 基地整定；反馈未接入前无实际作用） ----
constexpr Pid::Params kWheelPidInit{
    .kp = 8.0f, .ki = 0.5f, .kd = 0.0f,
    .integral_limit = 8000.0f,   // 3508 电流指令量级，M2 整定
    .output_limit = 16384.0f,    // C620 电流指令满量程，M2 确认
};
constexpr Pid::Params kFollowPidInit{
    .kp = 4.0f, .ki = 0.0f, .kd = 0.1f,
    .integral_limit = 1.5f,      // rad/s 量级
    .output_limit = cfg::kMaxWz, // 跟随修正量不超过底盘旋转限幅
};

} // namespace

Chassis::Chassis() {
  for (auto &p : wheel_pid_) p = Pid(kWheelPidInit);
  follow_pid_ = Pid(kFollowPidInit);
}

void Chassis::SetMode(ChassisMode m) {
  if (m == mode_) return;
  mode_ = m;
  ResetControllers(); // 切换瞬间清积分/微分，防输出跳变（bumpless）
}

void Chassis::ResetControllers() {
  for (auto &p : wheel_pid_) p.Reset();
  follow_pid_.Reset();
}

void Chassis::SetWheelFeedback(const float w_wheel[4]) {
  for (int i = 0; i < 4; ++i) wheel_feedback_[i] = w_wheel[i];
}

Chassis::Output Chassis::Update(const RcInput &rc, float theta, float dt) {
  // ---- 1) 摇杆 -> 云台系速度指令（暂定映射：左杆平移，拨轮手动旋转） ----
  kinematics::Velocity v_g{
      .vx = rc.lv * cfg::kMaxVx,   // 左杆上下 = 前后
      .vy = rc.lh * cfg::kMaxVy,   // 左杆左右 = 左右
      .wz = rc.wheel * cfg::kMaxWz // 拨轮 = 手动旋转分量
  };

  // ---- 2) 模式相关的旋转分量 ----
  switch (mode_) {
  case ChassisMode::kFollow:
    // 底盘 x 轴对齐云台 x 轴：θ>0（云台逆时针领先）时底盘需 wz>0 追赶。
    // Pid 输出 = kp·(target−feedback) = kp·(0−(−θ)) = +kp·θ，方向正确；
    // 微分作用于 −θ 上，等效对 θ 变化率产生阻尼。
    follow_pid_.SetTarget(0.0f);
    v_g.wz += follow_pid_.Compute(-kinematics::WrapPi(theta), dt);
    break;
  case ChassisMode::kSpin:
    v_g.wz += cfg::kSpinOmegaStart; // 常值自转叠加（首版 0.2 rad/s）
    break;
  case ChassisMode::kSeparate:
    break;
  }

  // ---- 3) 云台系 -> 底盘系 -> 麦轮解算 ----
  const kinematics::Velocity v_c =
      kinematics::RotateGimbalToChassis(v_g, theta);
  const kinematics::Velocity v_lim = kinematics::Clamp(v_c);

  Output out;
  out.wz_total = v_lim.wz;
  kinematics::MecanumInverse(v_lim, out.wheel_target);

  // ---- 4) 四轮速度环（反馈 M2 接入，当前占位 0） ----
  for (int i = 0; i < 4; ++i) {
    wheel_pid_[i].SetTarget(out.wheel_target[i]);
    out.motor_cmd[i] = wheel_pid_[i].Compute(wheel_feedback_[i], dt);
  }
  return out;
}
