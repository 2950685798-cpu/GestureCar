/**
  ******************************************************************************
  * @file    button.c
  * @brief   独立按键检测模块实现（HAL 库通用，可移植）
  *
  *          提供两种检测方式，按需选用：
  *            1. Button_ReadEvent()  按下事件：一次按下只返回一次 1（带消抖）
  *               -> 适合"按一下切换状态"：解锁/锁定、开关、模式切换
  *            2. Button_IsPressed()  实时状态：按住期间持续返回 1（无消抖）
  *               -> 适合"按住才执行"：点动、急停按钮
  *
  *          移植步骤：
  *            1. 把 button.c / button.h 拷贝到新工程
  *            2. 修改 button.h 顶部"用户配置区"的引脚宏
  *            3. 在 main 的初始化区调用一次 Button_Init()
  *            4. 在需要的地方调用 Button_ReadEvent() 或 Button_IsPressed()
  ******************************************************************************
  */
#include "button.h"

/**
  * @brief  按键引脚初始化：输入 + 上拉
  * @param  无
  * @retval 无
  * @note   引脚已在 CubeMX 里配置过（同参数）时，再调用一次也无副作用；
  *         移植时注意：如果该 GPIO 端口的时钟没开，需要先使能时钟
  *         （在 CubeMX 里配置了这个引脚就会自动开时钟）
  */
void Button_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin  = BUTTON_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = BUTTON_PULL;
  HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);
}

/**
  * @brief  按下事件检测：检测到一次"按下"返回 1，否则返回 0
  * @param  无
  * @retval 1 = 刚刚按下一次；0 = 没有新的按下
  * @note   带消抖（BUTTON_DEBOUNCE_MS）：电平变化后延时再确认一次，滤除抖动；
  *         只在"松开 -> 按下"的瞬间返回一次 1，按住不动不会重复返回，
  *         必须松开后再次按下，才会再次返回 1
  */
uint8_t Button_ReadEvent(void)
{
  static uint8_t lastState = !BUTTON_PRESSED_LEVEL;	/* 上次电平，初值 = 松开电平 */
  uint8_t now = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);

  if (now != lastState)								/* 电平有变化：消抖后再确认一次 */
  {
    HAL_Delay(BUTTON_DEBOUNCE_MS);
    now = HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN);
    if (now != lastState)							/* 确认电平真的变了 */
    {
      lastState = now;								/* 记录新电平 */
      if (now == BUTTON_PRESSED_LEVEL)
      {
        return 1;									/* 一次有效按下 */
      }
    }
  }
  return 0;
}

/**
  * @brief  实时按下状态：按键当前按住返回 1，松开返回 0
  * @param  无
  * @retval 1 = 当前按住；0 = 当前松开
  * @note   无消抖，直接读电平；适合"按住才执行"的场景（如点动、急停）
  */
uint8_t Button_IsPressed(void)
{
  return (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) == BUTTON_PRESSED_LEVEL) ? 1 : 0;
}
