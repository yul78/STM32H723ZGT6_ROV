#ifndef __FOC_H
#define __FOC_H

// #include "main.h"
#include "foc_types.h"
#include "smo.h"
#include "foc_config.h"
#include "foc_utils.h"
#include "foc_math.h"
#include "field_weakening.h"

#ifdef __cplusplus
extern "C"{
#endif

extern foc_handle_t FOC_Motor[MAX_MOTOR_NUM + 1];

foc_state_t Foc_Init(uint8_t motor_num, const foc_hal_t *hal_interface);
foc_state_t Foc_ParamInit(foc_handle_t *motor, const foc_hal_t *hal_interface);
foc_state_t Foc_Deinit(foc_handle_t *motor);
foc_state_t Foc_Loop(uint8_t motor_num);
foc_state_t Foc_Open_Loop(foc_handle_t *motor, float dt);
foc_state_t Foc_Open_Loop_Test(foc_handle_t *motor, float dt);
foc_state_t Foc_Close_Loop(foc_handle_t *motor, float dt);
foc_state_t Foc_Stop(uint8_t motor_num);
foc_state_t Foc_Set_Speed(uint8_t motor_num, float speed);



#ifdef __cplusplus
}
#endif

#endif // !__FOC_H


