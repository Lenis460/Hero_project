#include "pid.h"

namespace {

float ClampAbs(float v, float limit) {
  if (v > limit) return limit;
  if (v < -limit) return -limit;
  return v;
}

} // namespace

Pid::Pid(const Params &params) : params_(params) {}

void Pid::SetParams(const Params &params) { params_ = params; }

const Pid::Params &Pid::params() const { return params_; }

void Pid::SetTarget(float target) { target_ = target; }

float Pid::target() const { return target_; }

float Pid::Compute(float feedback, float dt) {
  if (dt <= 0.0f) {
    return last_output_; // 周期异常时保持上一帧输出
  }
  const float error = target_ - feedback;
  last_error_ = error;

  // 积分：累加 ki·e·dt，并对积分贡献限幅（抗饱和）
  integral_term_ = ClampAbs(integral_term_ + params_.ki * error * dt,
                            params_.integral_limit);

  // 微分先行：对反馈量微分，目标突变不冲击
  derivative_term_ = 0.0f;
  if (has_prev_feedback_) {
    derivative_term_ = -params_.kd * (feedback - prev_feedback_) / dt;
  }
  prev_feedback_ = feedback;
  has_prev_feedback_ = true;

  const float out =
      ClampAbs(params_.kp * error + integral_term_ + derivative_term_,
               params_.output_limit);
  last_output_ = out;
  return out;
}

void Pid::Reset() {
  integral_term_ = 0.0f;
  derivative_term_ = 0.0f;
  has_prev_feedback_ = false;
  last_output_ = 0.0f;
}
