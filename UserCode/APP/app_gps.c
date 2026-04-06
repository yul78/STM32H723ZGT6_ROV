#include "app_gps.h"

gps_data_t gps_data;

void APP_GPS_GetData(void);

/**
 * @brief 初始化函数
 */
void APP_GPS_Init(void)
{
    BSP_GPS_Init();
}

/**
 * @brief GPS数据处理任务函数，需在主循环中定期调用。
 */
void APP_GPS_Task(void)
{
    BSP_GPS_Task();
    APP_GPS_GetData();
}

/**
 * @brief 读取BSP缓存的最新RMC语句，并更新APP层经纬度结果。
 */
void APP_GPS_GetData(void)
{
	nmea_msg nmea = {0};
	uint8_t rmc_sentence[BSP_GPS_NMEA_SENTENCE_SIZE]; /* RMC语句临时快照。 */

	if (BSP_GPS_GetLatestRmcSentence(rmc_sentence, sizeof(rmc_sentence)) == 0U)
	{
		return;
	}

	MID_GPS_ParseRmc(&nmea, rmc_sentence);

	gps_data.latitude = (float)nmea.latitude / 100000.0f;
	gps_data.longitude = (float)nmea.longitude / 100000.0f;

	if (nmea.nshemi == 'S')
	{
		gps_data.latitude = -gps_data.latitude;
	}

	if (nmea.ewhemi == 'W')
	{
		gps_data.longitude = -gps_data.longitude;
	}
}