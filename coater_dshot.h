#ifndef __PICO_DSHOT_DSHOT_ENCODER_H__
#define __PICO_DSHOT_DSHOT_ENCODER_H__

#include <stdint.h>
#include "spin_coater.h"
#include "dshot_encoder.pio.h"

#define MIN_THROTTLE_COMMAND dshot_encoder_MIN_THROTTHLE_VALUE
#define MAX_THROTTLE_COMMAND dshot_encoder_MAX_THROTTHLE_VALUE
#define DSHOT_HEAVY_LOADED_IDLE_DUTY 100

bool dshot_init(uint16_t dshot_gpio);

void dshot_send_command(uint16_t c);

bool
set_dshot_safe(spin_coater_context_t* ctx, unsigned dshot);

#endif
