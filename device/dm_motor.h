#pragma once
// ============================================================================
// Dm4340 —— 达妙 DM-J4340-2EC CAN 位置-速度模式驱动
// 依据：《DM-J4340-2EC 减速电机使用说明书 V1.3》p12-18（../参考资料/开发板与电机手册/）
//
// 【手册已确认的协议事实】
//   - CAN 2.0 标准帧，默认 1 Mbps；反馈为问询式（收到 ID 匹配的帧才回发）
//   - 位置速度模式控制帧：ID = 0x100 + CAN_ID，D[0..3]=p_des float 小端，
//     D[4..7]=v_des float 小端（v_des 为梯形限速值，恒正）
//   - 使能帧：ID 同控制帧（0x100+ID），数据 FF FC FF FF FF FF FF FF
//   - 失能帧：FF FD FF FF FF FF FF FF；清错帧：FF FB ...；保存零点：FF FE ...
//   - 反馈帧（ID = Master ID）：D0 = ID | ERR<<4（ID 低 4 位，ERR 高 4 位）
//     D1..2 = POS 大端 int16；D3..4 高 12 位 = VEL；D4 低 4 位+D5 = T；
//     D6 = T_MOS ℃；D7 = T_Rotor(线圈) ℃
//   - 线性映射：位置 16 位默认 ±12.5 rad，速度/扭矩 12 位默认 ±30 rad/s / ±10 Nm
//     （范围可在调试助手改，需与实际设定一致）
//   - ERR：0=失能(上电默认) 1=使能 3/4/5=校准/传感器异常 8=超压 9=欠压
//     A=过流 B=MOS过温 C=线圈过温 D=通讯丢失 E=过载
//   - 上电后位置输出限定在 [-π, π] rad
//
// 【仍需现场确认（MarkProtocolConfirmed 前置）】
//   实物铭牌/电压版本、CAN_ID 与 Master ID 实际值、调试助手中模式寄存器(0x0A)=2
//   (位置速度)、Pos/Vel/Torque 映射范围设定值、CAN 通讯丢失保护超时配置
//
// 发送纪律：达妙"发一帧动一帧"，控制任务必须连续发送；调试少打断点。
// pitch 目标必须先过 pos_min/pos_max 软件限幅，不允许堵转找限位。
// ============================================================================

#include <cstdint>

namespace dm4340 {

constexpr uint16_t kPosVelCmdBase = 0x0100; // 位置速度模式控制帧基址

// ERR 语义（手册 p13）
constexpr uint8_t kErrDisabled = 0x0;
constexpr uint8_t kErrEnabled = 0x1;
// 3=输出轴校准异常 4=传感器异常 5=编码器校准异常 8..E=超压/欠压/过流/过温/丢通讯/过载
inline bool IsFaultErr(uint8_t err) { return err != kErrDisabled && err != kErrEnabled; }

struct Config {
  uint8_t can_id = 0x01;     // 待基地用调试助手核对
  uint16_t master_id = 0x00; // 反馈帧 ID，手册默认 0，待核对
  // 线性映射范围（默认值，须与调试助手参数页设定一致）
  float pos_range = 12.5f; // ±rad
  float vel_range = 30.0f; // ±rad/s
  float tq_range = 10.0f;  // ±N·m
  float pos_min = -1.0f;   // pitch 软件限幅，M4 按机械限位收紧
  float pos_max = 1.0f;
};

struct Feedback {
  uint8_t id = 0;  // 电机 CAN_ID 低 8 位（回包核对用）
  uint8_t err = kErrDisabled;
  float pos = 0.0f;    // rad（输出轴）
  float vel = 0.0f;    // rad/s
  float torque = 0.0f; // N·m
  uint8_t t_mos = 0;   // ℃
  uint8_t t_coil = 0;  // ℃（T_Rotor，线圈平均温度）
};

// 位置-速度目标帧（p_des 先软件限幅；返回帧数据 8 字节，帧 ID = 0x100+cfg.can_id）
uint8_t BuildPosVelFrame(const Config &cfg, float p_des, float v_des, uint8_t *out);

// 反馈帧解析（按手册 V1.3 布局；物理量经线性映射还原）
void ParseFeedback(const Config &cfg, const uint8_t *d, Feedback *out);

// ---- 命令帧（帧 ID 均为 0x100+can_id；数据 8 字节） ----
void BuildEnableFrame(uint8_t *out);   // FF FC FF FF FF FF FF FF
void BuildDisableFrame(uint8_t *out);  // FF FD FF FF FF FF FF FF
void BuildClearErrorFrame(uint8_t *out); // FF FB FF FF FF FF FF FF

class Driver {
public:
  explicit Driver(const Config &cfg) : cfg_(cfg) {}

  // 基地完成铭牌/CAN_ID/模式/映射范围核对后调用，打开发帧许可
  void MarkProtocolConfirmed() { protocol_confirmed_ = true; }
  bool protocol_confirmed() const { return protocol_confirmed_; }

  void OnFeedback(const uint8_t *data);
  const Feedback &feedback() const { return fb_; }
  bool Online() const { return online_; }
  // 发位置目标的三重前置：协议确认 + 反馈在线 + 当前处于使能态
  bool ReadyForCommand() const {
    return protocol_confirmed_ && online_ && fb_.err == kErrEnabled;
  }

  const Config &cfg() const { return cfg_; }

private:
  Config cfg_;
  Feedback fb_;
  bool online_ = false;
  bool protocol_confirmed_ = false;
};

} // namespace dm4340
