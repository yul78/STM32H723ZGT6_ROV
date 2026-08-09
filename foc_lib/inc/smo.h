#ifndef __SMO_H
#define __SMO_H

#include "foc_types.h"
#include "foc_config.h"
#include "foc_math.h"

#ifdef __cplusplus
extern "C" {
#endif

extern float angle_error;
foc_state_t SMO_Observer(foc_handle_t *motor, float dt, foc_mode_t mode);
foc_state_t SMO_PLL_loss(foc_handle_t *motor);

#ifdef __cplusplus
}
#endif

#endif


