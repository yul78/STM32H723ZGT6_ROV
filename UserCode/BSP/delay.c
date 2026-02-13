#include "stm32h7xx_hal.h"
// /**
//   * @brief  微秒级延时
//   * @param  xus 延时时长，范围：0~233015
//   * @retval 无
//   */
// void delay_us(uint32_t xus)
// {
// 	SysTick->LOAD = 72 * xus;				//设置定时器重装值
// 	SysTick->VAL = 0x00;					//清空当前计数值
// 	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
// 	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
// 	SysTick->CTRL = 0x00000004;				//关闭定时器
// }


/**
  * @brief  微秒级延时 (针对 H7 550MHz)
  * @param  xus 延时时长，单位微秒
  * @note   假设 SysTick 使用 HCLK/8 作为时钟源
  *         550MHz / 1 = 550MHz，
  *         每微秒计数 = 550
  * @retval 无
  */
void delay_us(uint32_t xus)
{
    // 安全检查
    if (xus == 0) return;
    
    // 每微秒的计数值 = (HCLK/8) / 1e6
    // 550MHz / 1 = 550MHz → 550 ticks/us
    uint32_t ticks_per_us = 550;  // 向上取整，更准确
    
    // 计算需要的总计数
    uint64_t total_ticks = (uint64_t)ticks_per_us * xus;
    
    // 限制在 24位最大值内
    if (total_ticks > 0xFFFFFF) {
        // 如果超过最大值，需要分多次延时
        uint32_t max_delay = 0xFFFFFF / ticks_per_us;
        while (xus > max_delay) {
            delay_us(max_delay);
            xus -= max_delay;
        }
        total_ticks = (uint64_t)ticks_per_us * xus;
    }
    
    // 设置 SysTick
    SysTick->LOAD = (uint32_t)total_ticks - 1;  // 减1是因为从 N-1 计数到 0
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |   // HCLK/8
                    SysTick_CTRL_ENABLE_Msk;       // 使能
    
    // 等待计数完成
    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
    
    // 关闭 SysTick
    SysTick->CTRL = 0;
}


/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
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
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void delay_s(uint32_t xs)
{
	while(xs--)
	{
		delay_ms(1000);
	}
} 
