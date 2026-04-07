#ifndef __FOC_HAL_H
#define __FOC_HAL_H

#include "stdint.h"

#ifdef __cplusplus
extern "C"{
#endif




typedef struct
{
    void (*pwm_start) (uint8_t num);                                                   // pwm开始
    void (*pwm_enable) (uint8_t num);                                                  // pwm使能                                  
    void (*pwm_disable) (uint8_t num);                                                 // pwm失能
    void (*pwm_set_duty) (uint8_t num, float du, float dv, float dw);                        // 设置占空比

    void (*drv_enable) (uint8_t num);                                                  // 驱动使能
    void (*drv_disable) (uint8_t num);                                                 // 驱动失能

    void (*adc_get_value) (uint8_t num, uint16_t *adc_u,  uint16_t *adc_v, uint16_t *adc_w);                  // 获取adc值

    void (*init) (uint8_t num);
}foc_hal_t;

extern const foc_hal_t foc_hal;



#ifdef __cplusplus
}
#endif



#endif


