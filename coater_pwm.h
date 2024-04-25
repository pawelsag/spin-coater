#ifndef __PICO_PWM_ENCODER_H__
#define __PICO_PWM_ENCODER_H__

#include "hardware/pwm.h"
#include "spin_coater.h"

/* When using pwm 60 hz, we can set duty between 6-12 */
/* When using pwm 50 hz, we can set duty between 5-10 */
#define PWM_INIT_FREQ 490
#define PWM_DUTY_MIN 49
#define PWM_DUTY_MAX 98
#define PWM_IDLE_DUTY PWM_DUTY_MIN
/* Normaly IDLE state for the engine is 49% pwm duty
   but since the engine is under load it starts spining from 53$
   instead of 49 % pwm duty cycle. This variable was added to speed up process
   of spining up the engine in automatic mode with timer */
#define PWM_HEAVY_LOADED_IDLE_DUTY 52

bool
pwm_set_freq_duty(uint32_t slice_num,
                  uint32_t chan,
                  uint32_t freq,
                  int duty_cycle);
bool
set_pwm_safe(spin_coater_context_t* ctx, unsigned duty);

#endif