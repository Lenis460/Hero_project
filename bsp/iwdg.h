#pragma once
// ============================================================================
// iwdg —— 独立看门狗封装（M1 在 CubeMX 中配置 IWDG 外设后填充实现）
//
// 铁律（task.md 安全门槛第 5 条）：
//   IwdgRefresh() 只允许在主控制循环"解码 -> SafetyGate::Update -> 指令输出/
//   失能"全部完成之后调用；禁止挂在 UART 回调或任何中断里。
//   当前为空实现（上板前不初始化 IWDG，程序不会被看门狗复位）。
// ============================================================================

namespace bsp {

void IwdgInit();    // M1: CubeMX 生成后按需补（Cube 的 MX_IWDG_Init 已在 main 调用，此函数预留给额外校验）
void IwdgRefresh(); // M1: 填充为 HAL_IWDG_Refresh(&hiwdg)

} // namespace bsp
