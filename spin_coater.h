#ifndef __PICO_SPIN_COATER_H__
#define __PICO_SPIN_COATER_H__
#include <stdint.h>
#include <stdio.h>
#include "pico/time.h"

#define DEBUG_printf printf

#define TCP_PORT 4242
#define BUF_SIZE 128

#define SPIN_COATER_LOGIC_GP_NUM 2

typedef enum
{
  STATE_DISCONNECTED = 0,
  STATE_LISTENING,
  STATE_CONNECTED,
} server_state_t;

typedef enum
{
  SPIN_IDLE = 0,
  SPIN_STARTED_WITH_TIMER,
  SPIN_STARTED_WITH_FORCE_VALUE,
  SPIN_SMOOTH_STOP_REQUESTED,
} spin_state_t;

typedef struct
{
  struct tcp_pcb* server_pcb;
  struct tcp_pcb* client_pcb;
  uint8_t recv_buffer[BUF_SIZE];
  uint16_t recv_len;
} tcp_server_context_t;

typedef struct
{
  tcp_server_context_t ctx;
  server_state_t connection_state;
#if SPIN_COATER_PWM_ENABLED
  uint32_t pwm_duty;
  unsigned int pwm_slice_num;
#elif SPIN_COATER_DSHOT_ENABLED
  uint32_t dshot_throttle_val;
#endif
  uint32_t set_rpm;
  spin_state_t spin_state;
  alarm_id_t timer_id;
} spin_coater_context_t;


#endif