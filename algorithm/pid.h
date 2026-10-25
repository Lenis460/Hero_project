#pragma once
// ============================================================================
// Pid —— 通用 PID 控制器（位置式，带积分限幅与输出限幅）
//
// 设计约定：
//   - Compute() 由控制任务以固定周期调用，dt [s] 由调用者传入；
//     本类不读取硬件时间 -> 纯计算，可在 Python/上位机离线复现
//   - 积分限幅作用于积分"贡献量"（ki·∫e·dt），单位与输出一致，
//     用于抗积分饱和（anti-windup）
//   - 微分先行：对反馈量微分（-d(feedback)/dt），目标值突变时不产生微分冲击
//   - 模式切换 / 使能 / 失能前后调用 Reset()，防止积分残留导致输出跳变
//
// 调参顺序（M2 速度环经验）：先 P（能跟上、略有振荡）-> 加 D 抑制振荡
// -> 最后加小量 I 消稳态误差；超调大先减 Ki 而不是减 Kp
// ============================================================================

class Pid {
public:
  struct Params {
    float kp;
    float ki;
    float kd;
    float integral_limit; // 积分项限幅（与输出同单位）
    float output_limit;   // 输出限幅
  };

  explicit Pid(const Params &params);

  void SetParams(const Params &params);
  const Params &params() const;

  void SetTarget(float target);
  float target() const;

  // 计算一次 PID；feedback 为被控量反馈，dt 为距上次调用的间隔 [s]
  float Compute(float feedback, float dt);

  // 清空积分与微分历史（保留目标值与参数）
  void Reset();

  // ---- 调试观察口（Ozone Watched Data / 串口打印用） ----
  float last_error() const { return last_error_; }
  float integral_term() const { return integral_term_; }
  float derivative_term() const { return derivative_term_; }
  float last_output() const { return last_output_; }

private:
  Params params_;
  float target_ = 0.0f;
  float integral_term_ = 0.0f; // 已乘 ki 并限幅后的积分贡献
  float prev_feedback_ = 0.0f;
  bool has_prev_feedback_ = false;
  float last_error_ = 0.0f;
  float derivative_term_ = 0.0f;
  float last_output_ = 0.0f;
};
