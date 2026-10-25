#include "iwdg.h"

namespace bsp {

// 空实现：IWDG 外设未配置前不产生任何动作。
// M1 基地配置 CubeMX IWDG 后，IwdgRefresh() 替换为 HAL_IWDG_Refresh(&hiwdg)。
void IwdgInit() {}

void IwdgRefresh() {}

} // namespace bsp
