#include "WaterADC.h"
#include "filter.h"

#define WATER_ZERO_THRESHOLD     0.10f   // 小于这个认为没水
#define WATER_DETECT_THRESHOLD   0.50f   // 明确进水电压
#define WATER_CONFIRM_COUNT      10      // 连续确认次数

extern uint16_t adc_value[2];
float voltage_value[2];

float lowpass_alpha = 0.1;

typedef struct
{
    uint8_t  confirm_cnt;
    uint8_t  detected;     // 0:未进水  1:已进水（锁存）
} water_sensor_t;

uint8_t water_sensor_process(water_sensor_t *sensor, float voltage);

/**
 * @brief 无人机进水检测函数
 * @retval 0表示未进水，1表示已进水
 */
uint8_t Water_Check(void)
{
    float voltage0 = (float)adc_value[0] / 4095.0f *3.3f;
    float voltage1 = (float)adc_value[1] / 4095.0f *3.3f;

    static float last_voltage0 = 0.0f;
    static float last_voltage1 = 0.0f;

    voltage0 = lowpass_filter(voltage0, &last_voltage0, lowpass_alpha);
    voltage1 = lowpass_filter(voltage1, &last_voltage1, lowpass_alpha);

    voltage_value[0] = voltage0;
    voltage_value[1] = voltage1;

    static water_sensor_t front_sensor = {0};
    static water_sensor_t rear_sensor = {0};

    uint8_t front_in = water_sensor_process(&front_sensor, voltage_value[0]);
    uint8_t rear_in = water_sensor_process(&rear_sensor, voltage_value[1]);

    if(front_in || rear_in)  //如果进水，立即上浮
    {
        return 1;
    }
    return 0;
}

uint8_t water_sensor_process(water_sensor_t *sensor, float voltage)
{
    /* 已锁存，直接返回 */
    if (sensor->detected)
        return 1;

    /* 低于零点阈值，清计数 */
    if (voltage < WATER_ZERO_THRESHOLD)
    {
        sensor->confirm_cnt = 0;
        return 0;
    }

    /* 超过进水阈值 */
    if (voltage > WATER_DETECT_THRESHOLD)
    {
        sensor->confirm_cnt++;

        if (sensor->confirm_cnt >= WATER_CONFIRM_COUNT)
        {
            sensor->detected = 1;
            return 1;
        }
    }
    else
    {
        sensor->confirm_cnt = 0;
    }

    return 0;
}
