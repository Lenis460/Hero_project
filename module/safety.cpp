#include "safety.h"

#include "config.h"

namespace safety {

SafetyGate::SafetyGate(float boot_delay_s) : boot_delay_s_(boot_delay_s) {}

const char *SafetyGate::StateName() const {
  switch (state_) {
  case State::kBootDelay: return "BOOT_DELAY";
  case State::kWaitRc: return "WAIT_RC";
  case State::kOutputAllowed: return "OUTPUT_ALLOWED";
  case State::kSafeStop: return "SAFE_STOP";
  default: return "?";
  }
}

bool SafetyGate::RcAlive(float now_s, uint32_t frame_count) {
  if (frame_count != last_frame_count_) {
    last_frame_count_ = frame_count;
    last_frame_time_s_ = now_s;
    return true;
  }
  if (last_frame_time_s_ < 0.0f) return false; // 从未收到过帧
  return (now_s - last_frame_time_s_) <= cfg::kRcTimeout;
}

State SafetyGate::Update(const Inputs &in) {
  // ---- 每周期全量重查（不沿用上一周期结论） ----
  uint16_t f = kFaultNone;
  if (!RcAlive(in.now_s, in.rc_frame_count)) f |= kFaultRcTimeout;
  if (in.estop) f |= kFaultEstop;
  if (!in.board_link_ok) f |= kFaultBoardLink;

  switch (state_) {
  case State::kBootDelay:
    if (in.now_s < boot_delay_s_) {
      faults_ = f; // 记录但不跳 SAFE_STOP（上电期遥控器可能还没开）
      break;
    }
    state_ = State::kWaitRc;
    [[fallthrough]];

  case State::kWaitRc:
    faults_ = f;
    if ((f & (kFaultRcTimeout | kFaultEstop | kFaultBoardLink)) == 0) {
      state_ = State::kOutputAllowed; // 各门通过即放行（门在输出态仍每周期复查）
    }
    break;

  case State::kOutputAllowed:
    faults_ = f;
    if ((f & (kFaultRcTimeout | kFaultEstop | kFaultBoardLink)) != 0) {
      state_ = State::kSafeStop; // 任一门失败 -> 闩锁
    }
    break;

  case State::kSafeStop:
    // 人工复位路径：故障源清除 + 摇杆回中，先回 kWaitRc 复核，再放行
    if (in.sticks_centered) {
      faults_ = f & ~kFaultNotCentered;
    } else {
      faults_ = f | kFaultNotCentered;
    }
    if ((faults_ & (kFaultRcTimeout | kFaultEstop | kFaultBoardLink)) == 0 &&
        in.sticks_centered) {
      state_ = State::kWaitRc;
    }
    break;
  }
  return state_;
}

} // namespace safety
