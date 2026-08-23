/* field_weakening.h */
#ifndef __FIELD_WEAKENING_H
#define __FIELD_WEAKENING_H

#include "foc_types.h"
#include "foc_config.h"

#ifdef __cplusplus
extern "C" {
#endif

void FOC_FieldWeakening(foc_handle_t *motor, float dt);

#ifdef __cplusplus
}
#endif
#endif
