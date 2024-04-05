/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Output PWM signals on pins 0 and 1

#include "pico/stdlib.h"
#include <stdint.h>
#include "hardware/pwm.h"
#include "hardware/platform_defs.h"
#include "hardware/clocks.h"
#include "pico/divider.h"
#include "pico/time.h"
#include <stdio.h>
#include "string.h"
#include "pico/stdlib.h"
#include <stdlib.h>

#define PWM_GP_NUM 2
/* When using pwm 60 hz, we can set duty between 6-12 */
/* When using pwm 50 hz, we can set duty between 5-10 */
#define PWM_INIT_FREQ 490
#define PWM_INIT_DUTY 49

uint64_t last_time;
uint64_t current_time_us;
uint64_t time_elapsed_us;

void gpio_callback(uint gpio, uint32_t events) {
  if (last_time == 0){
    last_time = time_us_64();
    return;
  }
  current_time_us = time_us_64();
  time_elapsed_us = current_time_us - last_time;
  last_time = current_time_us;
  printf("RPM: %llu\r\n", 60000000/time_elapsed_us);

}


int32_t pwm_set_freq_duty(uint32_t slice_num, uint32_t chan, uint32_t freq, 
                          int duty_cycle)
{

    uint8_t clk_divider = 0;
    uint32_t wrap = 0;
    uint32_t clock_div = 0;
    uint32_t clock = clock_get_hz(clk_sys);

    for(clk_divider = 1; clk_divider < UINT8_MAX; clk_divider++)
    {
        /* Find clock_division to fit current frequency */
        clock_div = div_u32u32( clock, clk_divider );
        wrap = div_u32u32(clock_div, freq);
        if (div_u32u32 (clock_div, UINT16_MAX) <= freq && wrap <= UINT16_MAX)
        {
            break;
        }
    }
    if(clk_divider < UINT8_MAX)
    {
        /* Only considering whole number division */
        pwm_set_clkdiv_int_frac(slice_num, clk_divider, 0);
        pwm_set_enabled(slice_num, true);
        pwm_set_wrap(slice_num, (uint16_t) wrap);
        pwm_set_chan_level(slice_num, chan, 
                          (uint16_t) div_u32u32((((uint16_t)(duty_cycle == 100? 
                          (wrap + 1) : wrap)) * duty_cycle), 100));
    }
    else
        return -2;

    return 1;
}

int main() {
    /// \tag::setup_pwm[]
    stdio_init_all();
    gpio_set_irq_enabled_with_callback(16, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Tell GPIO 0 and 1 they are allocated to the PWM
    gpio_set_function(PWM_GP_NUM, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num = pwm_gpio_to_slice_num(PWM_GP_NUM);

    // Set channel A output high for one cycle before dropping
    pwm_set_freq_duty(slice_num, PWM_CHAN_A, PWM_INIT_FREQ, PWM_INIT_DUTY);

    // Set initial B output high for three cycles before dropping
    // Set the PWM running
    pwm_set_enabled(slice_num, true);

    int i=0;
    size_t str_len = 0;
    char str [80];
    size_t duty = 0;
    while (true) {
        printf("Type PWM duty: ");
        scanf("%79s", str);
        str_len = strlen(str);
        if(str_len == 0)
            continue;
        duty = strtol(str, NULL, 10);
        printf("\n PWM duty is: %lu!\n", duty);
        pwm_set_freq_duty(slice_num, PWM_CHAN_A, PWM_INIT_FREQ, duty);
    }

    while(true) {
        sleep_ms(500);
    }
}
