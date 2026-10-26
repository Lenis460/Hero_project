#include "app.h"

#include <cstring>

#include "canbus.h" // bsp::CanInit / sniffer
#include "chassis.h"
#include "c620.h"
#include "config.h"
#include "dr16.h"
#include "mode.h"
#include "safety.h"
#include "usart.h" // huart3
#include "wdg.h"

// ---- C 接口的观察窗定义 ----
AppWatch app_watch;

namespace app {

remote_control::Dr16 g_dr16;
safety::SafetyGate g_gate(cfg::kBootDelay);
Chassis g_chassis;

// DMA 缓冲与重挂起（考核说明示例方式）
constexpr uint16_t kRxBufLen = remote_control::kRxBufLen;
uint8_t g_rx_buf[kRxBufLen];

float g_last_tick_s = 0.0f;

void RefreshWatchFromRc() {
  app_watch.rc_lh = g_dr16.rc_lh();
  app_watch.rc_lv = g_dr16.rc_lv();
  app_watch.rc_rh = g_dr16.rc_rh();
  app_watch.rc_rv = g_dr16.rc_rv();
  app_watch.rc_wheel = g_dr16.rc_wheel();
  app_watch.rc_l_sw = static_cast<uint8_t>(g_dr16.rc_l_switch());
  app_watch.rc_r_sw = static_cast<uint8_t>(g_dr16.rc_r_switch());
  app_watch.rc_frames = g_dr16.frame_count();
}

bool SticksCentered() {
  return g_dr16.rc_lh() == 0.0f && g_dr16.rc_lv() == 0.0f &&
         g_dr16.rc_rh() == 0.0f && g_dr16.rc_rv() == 0.0f &&
         g_dr16.rc_wheel() == 0.0f;
}

} // namespace app

extern "C" {

void AppInit(void) {
  bsp::WdgInit();
  bsp::CanInit();
  memset(&app_watch, 0, sizeof(app_watch));
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, app::g_rx_buf,
                               remote_control::kRxBufLen);
  app::g_last_tick_s = HAL_GetTick() * 0.001f;
}

void AppLoop(void) {
  using namespace app;

  // ---- 1) 时间 ----
  const float now_s = HAL_GetTick() * 0.001f;
  float dt = now_s - g_last_tick_s;
  if (dt <= 0.0f || dt > 0.05f) dt = 0.001f; // 周期异常保护（首圈/断点后）
  g_last_tick_s = now_s;

  // ---- 2) 安全判定 ----
  safety::Inputs in{};
  in.now_s = now_s;
  in.rc_frame_count = g_dr16.frame_count();
  in.estop = IsEstop(g_dr16.rc_l_switch());
  in.sticks_centered = SticksCentered();
  in.board_link_ok = true; // 单板阶段恒真；上下板链路 M2 前接入
  const safety::State st = g_gate.Update(in);

  // ---- 3) 控制计算（M1：只算不发） ----
  if (st == safety::State::kOutputAllowed) {
    RcInput rc{g_dr16.rc_lh(), g_dr16.rc_lv(), g_dr16.rc_rh(), g_dr16.rc_rv(),
               g_dr16.rc_wheel()};
    g_chassis.SetMode(MapChassisMode(g_dr16.rc_r_switch()));
    const Chassis::Output out = g_chassis.Update(rc, /*theta=*/0.0f, dt);
    for (int i = 0; i < 4; ++i) app_watch.wheel_target[i] = out.wheel_target[i];
    app_watch.wz_total = out.wz_total;
  } else {
    g_chassis.ResetControllers();
    for (int i = 0; i < 4; ++i) app_watch.wheel_target[i] = 0.0f;
    app_watch.wz_total = 0.0f;
  }

  // ---- 4) 观察量刷新 ----
  RefreshWatchFromRc();
  app_watch.gate_state = static_cast<uint8_t>(st);
  app_watch.gate_faults = g_gate.faults();
  app_watch.loop_count++;
  app_watch.tick_ms = HAL_GetTick();

  // ---- 5) 看门狗（铁律：主循环末尾、安全判定与输出处理完成后） ----
  bsp::WdgRefresh();
}

void AppOnUart3Rx(uint16_t size) {
  if (size == remote_control::kRcRxDataLen) {
    app::g_dr16.decode(app::g_rx_buf);
  }
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, app::g_rx_buf,
                               remote_control::kRxBufLen); // 重新挂起
}

void AppOnCanFrame(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t dlc) {
  (void)dlc;
  if (bus == 2 && id >= 0x201 && id <= 0x208) {
    // C620 反馈：ID = 0x200 + 电调 ID
    c620::Feedback fb[4];
    if (c620::OnFrame(static_cast<uint8_t>(id - 0x200), data, fb)) {
      for (int i = 0; i < 4; ++i) {
        app_watch.wheel_rpm[i] = fb[i].rotor_rpm;
        app_watch.wheel_temp[i] = fb[i].temp_c;
      }
    }
  }
  // bus 1（CAN1）：yaw/超电帧先由 sniffer 记录，协议确认后在此扩展解析
}

} // extern "C"

// ============================================================================
// HAL 弱符号回调重写（中断上下文）
// ============================================================================
extern "C" {

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == &huart3) {
    AppOnUart3Rx(Size);
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) == HAL_OK) {
    const uint8_t bus = (hcan->Instance == CAN1) ? 1 : 2;
    bsp::CanOnFrame(hcan, header, data);
    AppOnCanFrame(bus, header.StdId, data,
                  static_cast<uint8_t>(header.DLC));
  }
}

} // extern "C"
