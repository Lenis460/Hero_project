#pragma once
// ============================================================================
// can —— 双路 CAN 底板支持（滤波器/启动/中断收帧）
//
// 总线分配（实车拓扑 2026-10-25）：
//   CAN1（PD0/PD1，滑环共享总线）：RX-only 监听 yaw DM 反馈 + 超级电容；
//     本工程 M1 不在 CAN1 发任何帧（板间协议未定稿，ID 避让未完成）
//   CAN2（PB12/PB13，轮电机总线）：4×M3508+C620；M1 同样只收不发
//
// 收帧路径：HAL_CAN_RxFifo0MsgPendingCallback（app.cpp 重写）→ bsp::CanOnFrame
// M1 抓帧：CanSniffer 记录每路最近 16 帧原始内容 + 已知 ID 计数，
//   Ozone Watched Data 观察即可摸清总线上有哪些设备
// ============================================================================

#include <cstdint>

#include <can.h> // Cube 生成：hcan1/hcan2（用尖括号走 -I 搜索，避免引号包含命中本文件自身）

namespace bsp {

// 启动双路 CAN：配全接收滤波器（全部进 FIFO0）→ CAN_Start
void CanInit();

// 每路总线的收帧统计（Ozone Watch 观察用；volatile：中断写、主循环读）
struct BusStats {
  volatile uint32_t frame_count = 0;      // 累计收帧数
  volatile uint32_t error_count = 0;      // HAL 错误回调计数
};

// M1 抓帧器：每路环形记录最近 16 帧（中断写，主循环/Ozone 只读）
struct Sniffer {
  static constexpr uint8_t kDepth = 16;
  struct Slot {
    volatile uint32_t id;   // 标准 11 位 ID
    volatile uint8_t dlc;
    volatile uint8_t data[8];
  };
  Slot ring[kDepth];
  volatile uint8_t head = 0; // 下一写入位置
};

BusStats &can1_stats();
BusStats &can2_stats();
Sniffer &can1_sniffer();
Sniffer &can2_sniffer();

// CAN 接收中断调用（HAL_CAN_RxFifo0MsgPendingCallback 内部转发）
void CanOnFrame(CAN_HandleTypeDef *hcan, const CAN_RxHeaderTypeDef &header,
                const uint8_t *data);

} // namespace bsp
