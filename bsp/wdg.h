#pragma once
// ============================================================================
// wdg —— 独立看门狗封装（IWDG 由 CubeMX 配置：Prescaler 32, Reload 500 ≈ 500ms）
//
// 铁律（task.md 安全门槛第 5 条）：
//   WdgRefresh() 只允许在主控制循环"解码 -> SafetyGate::Update -> 指令输出/
//   失能"全部完成之后调用一次；禁止挂在 UART/CAN 回调或任何中断里。
//
// 调试注意：WdgInit() 已冻结 IWDG 在调试暂停时计数（否则断点 >500ms 就复位）。
//   因此 M1-6 的"看门狗杀程序"验证必须在无调试器独立运行时做。
// ============================================================================

namespace bsp {

void WdgInit();    // 调试器暂停期间冻结 IWDG（仅影响调试，不影响独立运行）
void WdgRefresh(); // HAL_IWDG_Refresh(&hiwdg)，只在主循环末尾调用

} // namespace bsp
