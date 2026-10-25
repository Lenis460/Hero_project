#pragma once
// ============================================================================
// SafetyGate —— 安全状态机（task.md「第一次电机输出前的安全门槛」的代码化）
//
// 状态机（对应 task.md 生命周期，FEEDBACK_ONLY 并入 kWaitRc，待 IMU 上线后
// 在 kWaitRc 内扩展零漂采集）：
//   kBootDelay    上电 kBootDelay 秒，只收反馈，禁止使能/运动指令
//   kWaitRc       等待遥控器有效帧（+未来：IMU 零漂完成、上下板链路正常）
//   kOutputAllowed 全部安全门当周期通过，允许输出
//   kSafeStop     闩锁态：急停/失联/断链触发，清零目标并走失能路径；
//                 不自动恢复，需条件清除 + 摇杆回中 + 拨杆回安全位（人工复位）
//
// 规则（每个控制周期 Update 全量重查，不沿用上一周期结论）：
//   1. 急停最高优先级；2. 遥控器 kRcTimeout 内 frame_count 无增长 = 失联；
//   3. 上下板链路（若启用）超时 = 失联；4. 任一门失败 -> kSafeStop；
//   5. kOutputAllowed 下每周期重查所有门
//
// IWDG 配套规则：主循环顺序固定为 解码 -> SafetyGate::Update -> 指令输出/失能
// -> IwdgRefresh()；看门狗刷新只能发生在输出处理完成之后（bsp/iwdg.h）
// ============================================================================

#include <cstdint>

namespace safety {

enum class State : uint8_t {
  kBootDelay = 0,
  kWaitRc,
  kOutputAllowed,
  kSafeStop,
};

// 故障位（可叠加；faults() 供串口/Ozone 观察）
enum FaultBits : uint16_t {
  kFaultNone = 0,
  kFaultRcTimeout = 1u << 0,
  kFaultEstop = 1u << 1,
  kFaultBoardLink = 1u << 2,
  kFaultNotCentered = 1u << 3, // 复位条件未满足（摇杆未回中）
};

struct Inputs {
  float now_s;              // 当前时间 [s]，由调用者传入（假时间可测）
  uint32_t rc_frame_count;  // remote_control::Dr16::frame_count()
  bool estop;               // 急停拨杆触发中
  bool sticks_centered;     // 四摇杆+拨轮均回中（|v|<死区）
  bool board_link_ok;       // 单板结构恒填 true；上下板结构由链路监测填
};

class SafetyGate {
public:
  explicit SafetyGate(float boot_delay_s);

  // 每控制周期调用一次；返回当前状态
  State Update(const Inputs &in);

  State state() const { return state_; }
  bool OutputAllowed() const { return state_ == State::kOutputAllowed; }
  uint16_t faults() const { return faults_; }
  const char *StateName() const;

private:
  bool RcAlive(float now_s, uint32_t frame_count);

  const float boot_delay_s_;
  State state_ = State::kBootDelay;
  uint16_t faults_ = kFaultNone;

  // 遥控器帧进度跟踪
  uint32_t last_frame_count_ = 0;
  float last_frame_time_s_ = -1.0f; // -1 = 从未收到过帧
};

} // namespace safety
