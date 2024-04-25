#include "coater_pwm.h"
#include "hardware/clocks.h"
#include "pico/divider.h"

bool
pwm_set_freq_duty(uint32_t slice_num,
                  uint32_t chan,
                  uint32_t freq,
                  int duty_cycle)
{

  uint8_t clk_divider = 0;
  uint32_t wrap = 0;
  uint32_t clock_div = 0;
  uint32_t clock = clock_get_hz(clk_sys);

  for (clk_divider = 1; clk_divider < UINT8_MAX; clk_divider++) {
    /* Find clock_division to fit current frequency */
    clock_div = div_u32u32(clock, clk_divider);
    wrap = div_u32u32(clock_div, freq);
    if (div_u32u32(clock_div, UINT16_MAX) <= freq && wrap <= UINT16_MAX) {
      break;
    }
  }
  if (clk_divider < UINT8_MAX) {
    /* Only considering whole number division */
    pwm_set_clkdiv_int_frac(slice_num, clk_divider, 0);
    pwm_set_enabled(slice_num, true);
    pwm_set_wrap(slice_num, (uint16_t)wrap);
    pwm_set_chan_level(
      slice_num,
      chan,
      (uint16_t)div_u32u32(
        (((uint16_t)(duty_cycle == 100 ? (wrap + 1) : wrap)) * duty_cycle),
        100));
  } else
    return false;

  return true;
}

bool
set_pwm_safe(spin_coater_context_t* ctx, unsigned duty)
{
  if (duty < PWM_DUTY_MIN || duty > PWM_DUTY_MAX)
    return false;

  ctx->pwm_duty = duty;
  DEBUG_printf("\n PWM set to: %lu!\n", duty);
  return pwm_set_freq_duty(ctx->pwm_slice_num, PWM_CHAN_A, PWM_INIT_FREQ, duty);
}