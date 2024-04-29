#ifndef __PICO_DSHOT_DSHOT_ENCODER_H__
#define __PICO_DSHOT_DSHOT_ENCODER_H__

#include <stdint.h>
#include "spin_coater.h"

#define MIN_THROTTLE_COMMAND 48
#define MAX_THROTTLE_COMMAND 2047
#define DSHOT_HEAVY_LOADED_IDLE_DUTY 100

// Init PIO, but do not output data yet
// If enable_repeat is true, the PIO will continuously output the last provided value at
// 8000 frames per second.
// Warning: enable_repeat should only be used with the watchdog enabled, otherwise it is
// unsafe as the PIO will carry on sending dshot frames even if the CPU is stuck.
bool dshot_init(uint16_t dshot_gpio, bool enable_repeat);

// Note: enable_repeat applies to the latest value from any send function.
// When enable_repeat is false, each send function call triggers a single dshot frame

// Send a raw DShot command
void dshot_send_command(uint16_t c);

// Send throttle command once, for specified throttle in range [0, 1]
void dshot_send_throttle(double t);

// Stop generating output (until next send command)
// It is not necessary to call this when enable_repeat is false
void dshot_stop();

bool
set_dshot_safe(spin_coater_context_t* ctx, unsigned dshot);

#endif
