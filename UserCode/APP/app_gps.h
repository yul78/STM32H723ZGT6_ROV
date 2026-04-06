#ifndef __APP_GPS_H
#define __APP_GPS_H

#include "bsp_gps.h"
#include "mid_gps.h"

typedef struct 
{
    float latitude;     // 当前纬度，单位为度
    float longitude;    // 当前经度，单位为度
}gps_data_t;

extern gps_data_t gps_data; // APP层对外提供的GPS业务数据

void APP_GPS_Init(void);
void APP_GPS_Task(void);

#endif