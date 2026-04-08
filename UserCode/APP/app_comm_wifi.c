#include "app_comm_wifi.h"

void APP_WIFI_Transmit_GPSData(void)
{
	char buffer[64];
	snprintf(buffer, sizeof(buffer), "lat:%.6f, lng:%.6f\n", gps_data.latitude, gps_data.longitude);
	HAL_UART_Transmit(&WIFI_huart, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}