#include "wdg.h"

#include "iwdg.h" // Cube 生成的 Core/Inc/iwdg.h，提供 hiwdg

namespace bsp {

void WdgInit() {
  // 调试暂停（断点）时冻结看门狗计数；正式验证看门狗杀程序时不接调试器
  __HAL_DBGMCU_FREEZE_IWDG();
}

void WdgRefresh() { HAL_IWDG_Refresh(&hiwdg); }

} // namespace bsp
