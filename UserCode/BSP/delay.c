#include "stm32h7xx_hal.h"

/**
  * @brief  微秒级延时
  * @param  us 延时时长，单位微秒

  * @retval 无
  */
void delay_us(uint32_t us)
{
    if (us == 0) return;

    uint32_t ticks_per_us = SystemCoreClock / 1000000U;  // 自动计算
    uint32_t total_ticks  = us * ticks_per_us;

    uint32_t start = SysTick->VAL;
    uint32_t reload = SysTick->LOAD + 1;   // 一周期总tick数

    while (total_ticks > 0)
    {
        uint32_t now = SysTick->VAL;
        uint32_t elapsed;

        if (start >= now)
        {
            elapsed = start - now;
        }
        else
        {
            elapsed = start + (reload - now);
        }

        if (elapsed >= total_ticks)
            break;

        total_ticks -= elapsed;
        start = now;
    }
}


/**
  * @brief  毫秒级延时
  * @param  xms 延时时长
  * @retval 无
  */
void delay_ms(uint32_t xms)
{
	while(xms--)
	{
		delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长
  * @retval 无
  */
void delay_s(uint32_t xs)
{
	while(xs--)
	{
		delay_ms(1000);
	}
} 
