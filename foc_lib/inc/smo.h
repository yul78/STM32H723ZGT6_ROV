#ifndef __SMO_H
#define __SMO_H

#include "foc_types.h"
#include "foc_config.h"
#include "foc_math.h"

#ifdef __cplusplus
extern "C" {
#endif

extern float angle_error;
void BEMF_Observer(foc_handle_t *motor, float dt, foc_mode_t mode);
void SMO_Observer(foc_handle_t *motor, float dt, foc_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif


