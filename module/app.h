#pragma once
// ============================================================================
// app —— M1 主控胶水层（下板）【C 兼容头：可被 main.c 直接包含】
//
// 主循环固定顺序（task.md 安全铁律）：
//   AppLoop: 取时间 -> SafetyGate::Update -> （放行时 Chassis 更新，M1 仅计算
//   不发送）-> 非输出态清零复位 -> WdgRefresh()
//
// 中断钩子（HAL 弱符号回调在 app.cpp 内重写）：
//   UART3 DMA 空闲事件 -> AppOnUart3Rx  -> dr16.decode（Size==18 才解）
//   CAN1/CAN2 FIFO0    -> AppOnCanFrame -> sniffer + C620 反馈解析
//
// M1 输出策略：无论状态如何都不向任何电机发帧（CAN 发送代码未接入，天然满足
// "第一次电机输出前安全门槛"）；wheel_target 仅供 Ozone 观察核对解算方向。
//
// 调试：app_watch 结构体 volatile，Ozone Watched Data 添加 &app_watch 实时观察。
// ============================================================================

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Ozone 实时观察窗（Watch 窗口添加 &app_watch）
typedef struct {
  // 遥控器（-1~1）
  volatile float rc_lh, rc_lv, rc_rh, rc_rv, rc_wheel;
  volatile uint8_t rc_l_sw, rc_r_sw; // 1=上 3=中 2=下 0=无效
  volatile uint32_t rc_frames;       // DBus 累计帧数（判断失联）

  // 安全状态机：0=BOOT 1=WAIT_RC 2=OUTPUT_ALLOWED 3=SAFE_STOP
  volatile uint8_t gate_state;
  volatile uint16_t gate_faults;

  // 底盘解算结果（M1 仅观察，不发送）
  volatile float wheel_target[4]; // FL,FR,RL,RR 轮端 rad/s
  volatile float wz_total;

  // C620 反馈（按 FL,FR,RL,RR；CAN2 上电即有）
  volatile float wheel_rpm[4]; // 转子 rpm
  volatile uint8_t wheel_temp[4];

  // 时间
  volatile uint32_t loop_count;
  volatile uint32_t tick_ms;
} AppWatch;

extern AppWatch app_watch;

void AppInit(void); // 起流 UART DMA、启动 CAN、初始化各模块
void AppLoop(void); // 主循环体（while(1) 内调用）

// ---- 中断侧钩子（由 app.cpp 内的 HAL 回调调用） ----
void AppOnUart3Rx(uint16_t size);
void AppOnCanFrame(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t dlc);

#ifdef __cplusplus
}
#endif
