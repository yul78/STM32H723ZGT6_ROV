#include "app_thrusters.h"

void APP_Thrusters_Init(void)
{
    Foc_Init(1, &foc_hal);
    Foc_Init(2, &foc_hal);
}