#include "WaterADC.h"
#include "filter.h"

#define WATER_ZERO_THRESHOLD     0.10f   // 小于这个认为没水
#define WATER_DETECT_THRESHOLD   0.50f   // 明确进水电压
#define WATER_CONFIRM_COUNT      10      // 连续确认次数

uint16_t adc_value[2];
float voltage_value[2];

float lowpass_alpha = 0.1; //低通滤波参数

/* 水量检测传感器的结构体变量 */
typedef struct
{
    uint8_t  confirm_cnt;  // 连续确认计数
    uint8_t  detected;     // 0:未进水  1:已进水（锁存）
} water_sensor_t;

uint8_t Water_Sensor_Process(water_sensor_t *sensor, float voltage);

/**
 * @brief 水量传感器ADC初始化函数
 */
void WaterADC_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);                   //ADC校准
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc_value, sizeof(adc_value)/sizeof(adc_value[0]));  //开启ADC DMA转换
}
/**
 * @brief 无人机进水检测函数
 * @retval 0表示未进水，1表示已进水
 */
uint8_t Water_Check(void)
{
    voltage_value[0] = (float)adc_value[0] / 4095.0f *3.3f;
    voltage_value[1] = (float)adc_value[1] / 4095.0f *3.3f;

    static float last_voltage0 = 0.0f;
    static float last_voltage1 = 0.0f;

    voltage_value[0] = lowpass_filter(voltage_value[0], &last_voltage0, lowpass_alpha);
    voltage_value[1] = lowpass_filter(voltage_value[1], &last_voltage1, lowpass_alpha);

    last_voltage0 = voltage_value[0];
    last_voltage1 = voltage_value[1];

    static water_sensor_t front_sensor = {0};
    static water_sensor_t rear_sensor = {0};

    uint8_t front_in = Water_Sensor_Process(&front_sensor, voltage_value[0]);
    uint8_t rear_in = Water_Sensor_Process(&rear_sensor, voltage_value[1]);

    if(front_in || rear_in)  //如果进水，立即上浮
    {
        return 1;
    }
    return 0;
}

/**
 * @brief 水量传感器处理函数
 * @param sensor 传感器结构体指针
 * @param voltage 传感器获取的电压值
 * @retval 0表示未进水，1表示已进水
 */
uint8_t Water_Sensor_Process(water_sensor_t *sensor, float voltage)
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
