#include "filter.h"

/**
 * @brief 一阶低通滤波
 * @param input  当前采样值
 * @param last   上一次滤波结果
 * @param alpha  滤波系数(0~1)，越小越平滑
 * @return       滤波后的结果
 */
float lowpass_filter(float input, float *last, float alpha)
{
    *last = (*last) + alpha * (input - (*last));
    return *last;
}