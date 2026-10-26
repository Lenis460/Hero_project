#include "canbus.h"

namespace bsp {

namespace {

BusStats g_can1_stats, g_can2_stats;
Sniffer g_can1_sniff, g_can2_sniff;

void ConfigureFilter(CAN_HandleTypeDef *hcan, uint8_t bank) {
  CAN_FilterTypeDef f{};
  f.FilterBank = bank;
  f.FilterMode = CAN_FILTERMODE_IDMASK;
  f.FilterIdHigh = 0x0000;
  f.FilterIdLow = 0x0000;
  f.FilterMaskIdHigh = 0x0000; // Mask 全 0 = 全部接收
  f.FilterMaskIdLow = 0x0000;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation = CAN_FILTER_ENABLE;
  f.SlaveStartFilterBank = 14; // CAN2（从机）从 bank 14 起
  if (HAL_CAN_ConfigFilter(hcan, &f) != HAL_OK) {
    Error_Handler();
  }
}

} // namespace

void CanInit() {
  ConfigureFilter(&hcan1, 0);  // CAN1（主机）用 bank 0
  ConfigureFilter(&hcan2, 14); // CAN2（从机）用 bank 14
  if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
  if (HAL_CAN_Start(&hcan2) != HAL_OK) Error_Handler();
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

BusStats &can1_stats() { return g_can1_stats; }
BusStats &can2_stats() { return g_can2_stats; }
Sniffer &can1_sniffer() { return g_can1_sniff; }
Sniffer &can2_sniffer() { return g_can2_sniff; }

void CanOnFrame(CAN_HandleTypeDef *hcan, const CAN_RxHeaderTypeDef &header,
                const uint8_t *data) {
  if (hcan->Instance == CAN1) {
    Sniffer &s = g_can1_sniff;
    Sniffer::Slot &slot = s.ring[s.head];
    slot.id = header.StdId;
    slot.dlc = header.DLC;
    for (uint8_t i = 0; i < 8; ++i) {
      slot.data[i] = (i < header.DLC) ? data[i] : 0;
    }
    s.head = (s.head + 1) % Sniffer::kDepth;
    ++g_can1_stats.frame_count;
  } else if (hcan->Instance == CAN2) {
    Sniffer &s = g_can2_sniff;
    Sniffer::Slot &slot = s.ring[s.head];
    slot.id = header.StdId;
    slot.dlc = header.DLC;
    for (uint8_t i = 0; i < 8; ++i) {
      slot.data[i] = (i < header.DLC) ? data[i] : 0;
    }
    s.head = (s.head + 1) % Sniffer::kDepth;
    ++g_can2_stats.frame_count;
  }
}

} // namespace bsp
