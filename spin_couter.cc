/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Output PWM signals on pins 0 and 1

#include <stdint.h>
#include <stdio.h>
#include "string.h"

#include "pico/divider.h"
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#define TCP_PORT 4242
#define DEBUG_printf printf
#define BUF_SIZE 128
#define TEST_ITERATIONS 10
#define POLL_TIME_S 5

#define PWM_GP_NUM 2
/* When using pwm 60 hz, we can set duty between 6-12 */
/* When using pwm 50 hz, we can set duty between 5-10 */
#define PWM_INIT_FREQ 490
#define PWM_INIT_DUTY 49

uint64_t last_time;
uint64_t current_time_us;
uint64_t time_elapsed_us;
uint32_t current_rpm;

typedef enum {
  STATE_DISCONNECTED = 0,
  STATE_LISTENING,
  STATE_CONNECTED,
} server_state_t;

typedef struct {
  struct tcp_pcb* server_pcb;
  struct tcp_pcb* client_pcb;
  uint8_t         recv_buffer[BUF_SIZE];
  u16_t           recv_len;
} tcp_server_context_t;

typedef struct {
  tcp_server_context_t ctx;
  server_state_t  state;
  uint32_t pwm_duty;
  uint32_t spin_time;
} spin_couter_context_t;

static err_t
tcp_server_send_data(spin_couter_context_t* ctx,
                     struct tcp_pcb*       tpcb,
                     const uint8_t*              data,
                     u16_t                 size);

template<typename Callable>
static void
command_handler(spin_couter_context_t* ctx, uint8_t* buffer, Callable &feedback);

void gpio_callback(uint gpio, uint32_t events) {

  if (last_time == 0){
    last_time = time_us_64();
    return;
  }
  const uint64_t microseconds_in_minute = 60000000;
  const uint64_t number_of_wholes = 2;

  current_time_us = time_us_64();
  time_elapsed_us = current_time_us - last_time;
  last_time = 0;
  current_rpm = (microseconds_in_minute/time_elapsed_us)/number_of_wholes;
}


bool pwm_set_freq_duty(uint32_t slice_num, uint32_t chan, uint32_t freq, 
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
        return false;

    return true;
}
bool set_pwm_safe(spin_couter_context_t* ctx, unsigned duty)
{
  if(duty < 49 || duty > 98)
    return false;

  ctx->pwm_duty = duty;
  DEBUG_printf("\n PWM set to: %lu!\n", duty);
  return pwm_set_freq_duty(duty, PWM_CHAN_A, PWM_INIT_FREQ, duty);
}


static spin_couter_context_t* tcp_server_init(void) {
    spin_couter_context_t *ctx = (spin_couter_context_t*)calloc(1, sizeof(spin_couter_context_t));
    if (!ctx) {
        DEBUG_printf("failed to allocate state\n");
        return NULL;
    }
    return ctx;
}

static err_t tcp_server_close(void *arg) {
    spin_couter_context_t *sp_ctx = (spin_couter_context_t*)arg;
    tcp_server_context_t *ctx = &sp_ctx->ctx;
    err_t err = ERR_OK;
    if (ctx->client_pcb != NULL) {
        tcp_arg(ctx->client_pcb, NULL);
        tcp_poll(ctx->client_pcb, NULL, 0);
        tcp_sent(ctx->client_pcb, NULL);
        tcp_recv(ctx->client_pcb, NULL);
        tcp_err(ctx->client_pcb, NULL);
        err = tcp_close(ctx->client_pcb);
        if (err != ERR_OK) {
            DEBUG_printf("close failed %d, calling abort\n", err);
            tcp_abort(ctx->client_pcb);
            err = ERR_ABRT;
        }
        ctx->client_pcb = NULL;
    }
    if (ctx->server_pcb) {
        tcp_arg(ctx->server_pcb, NULL);
        tcp_close(ctx->server_pcb);
        ctx->server_pcb = NULL;
    }
    return err;
}
static err_t
tcp_server_sent(void* ctx_, struct tcp_pcb* tpcb, u16_t len)
{
  spin_couter_context_t* ctx = (spin_couter_context_t*) ctx_;

  return ERR_OK;
}

static err_t
tcp_server_send_data(spin_couter_context_t* ctx,
                     struct tcp_pcb*       tpcb,
                     const uint8_t*        data,
                     u16_t                 size)
{
  return tcp_write(tpcb, data, size, TCP_WRITE_FLAG_COPY);
}


static void
tcp_command_handler(spin_couter_context_t* sp_ctx, struct tcp_pcb* tpcb)
{
  auto feedback_send = [&sp_ctx, &tpcb](const char* msg, const size_t msg_len){
    tcp_server_send_data(sp_ctx,
                         tpcb,
                         (const uint8_t*)msg,
                         msg_len);
  };

  command_handler(sp_ctx, sp_ctx->ctx.recv_buffer, feedback_send);
}

template<typename Callable>
static void
command_handler(spin_couter_context_t* ctx, uint8_t* buffer, Callable &feedback)
{
  unsigned arg;
  char     str_arg[BUF_SIZE];

  if (buffer[0] == '\n') return;

  if (sscanf((const char*)buffer, "pwm: %u", &arg) == 1) {
    const bool res = set_pwm_safe(ctx, arg);
    if(!res)
    {
      DEBUG_printf("Chaning PWM duty failed\n");
      return;
    }
    char msg[BUF_SIZE];
    const size_t msg_len = snprintf(msg, sizeof(msg), "pwm: %u\n", ctx->pwm_duty);
    feedback(msg, msg_len);
  }
}

err_t
tcp_server_recv(void* ctx_, struct tcp_pcb* tpcb, struct pbuf* p, err_t err)
{
  spin_couter_context_t* sp_ctx = (spin_couter_context_t*)ctx_;
  tcp_server_context_t* ctx = &sp_ctx->ctx;

  if (!p) {
    tcp_server_close(sp_ctx);
    return err;
  }

  if (p->tot_len > 0) {
    DEBUG_printf(
      "tcp_server_recv %d/%d err %d\n", p->tot_len, ctx->recv_len, err);

    // Receive the buffer
    const uint16_t buffer_left = BUF_SIZE - ctx->recv_len;
    ctx->recv_len +=
      pbuf_copy_partial(p,
                        ctx->recv_buffer + ctx->recv_len,
                        p->tot_len > buffer_left ? buffer_left : p->tot_len,
                        0);

    tcp_recved(tpcb, p->tot_len);
  }

  pbuf_free(p);
  ctx->recv_buffer[ctx->recv_len] = '\0';
  // Echo back for debugging
  tcp_command_handler(sp_ctx, tpcb);

  ctx->recv_len = 0;

  return ERR_OK;
}

static void
tcp_server_err(void* ctx_, err_t err)
{
  spin_couter_context_t* sp_ctx = (spin_couter_context_t*)ctx_;

  if (err != ERR_ABRT) {
    DEBUG_printf("tcp_client_err_fn %d\n", err);
    sp_ctx->state = STATE_DISCONNECTED;
    tcp_server_close(sp_ctx);
  }
}

static err_t
tcp_server_accept(void* ctx_, struct tcp_pcb* client_pcb, err_t err)
{
  spin_couter_context_t* sp_ctx = (spin_couter_context_t*)ctx_;

  if (err != ERR_OK || client_pcb == NULL) {
    DEBUG_printf("Failure in accept\n");
    sp_ctx->state = STATE_DISCONNECTED;
    tcp_server_close(sp_ctx);
    return ERR_VAL;
  }

  DEBUG_printf("Client connected\n");

  sp_ctx->ctx.client_pcb = client_pcb;
  tcp_arg(client_pcb, sp_ctx);
  tcp_sent(client_pcb, tcp_server_sent);
  tcp_recv(client_pcb, tcp_server_recv);
  tcp_err(client_pcb, tcp_server_err);

  sp_ctx->state = STATE_CONNECTED;
  return ERR_OK;
}

static bool
tcp_server_open(void* ctx_)
{
  spin_couter_context_t* sp_ctx = (spin_couter_context_t*)ctx_;
  tcp_server_context_t* ctx = &sp_ctx->ctx;
  DEBUG_printf("Starting server at %s on port %u\n",
               ip4addr_ntoa(netif_ip4_addr(netif_list)),
               TCP_PORT);

  struct tcp_pcb* pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);
  if (!pcb) {
    DEBUG_printf("failed to create pcb\n");
    return false;
  }

  err_t err = tcp_bind(pcb, NULL, TCP_PORT);
  if (err) {
    DEBUG_printf("failed to bind to port %u\n", TCP_PORT);
    return false;
  }

  ctx->server_pcb = tcp_listen_with_backlog(pcb, 1);
  if (!ctx->server_pcb) {
    DEBUG_printf("failed to listen\n");
    if (pcb) {
      tcp_close(pcb);
    }
    return false;
  }

  sp_ctx->state = STATE_LISTENING;
  tcp_arg(ctx->server_pcb, sp_ctx);
  tcp_accept(ctx->server_pcb, tcp_server_accept);

  return true;
}


void run_tcp_server_test(void) {
    spin_couter_context_t* sp_ctx = tcp_server_init();
    if (!sp_ctx) {
        return;
    }
    if (!tcp_server_open(sp_ctx)) {
        return;
    }

    absolute_time_t update_deadline = make_timeout_time_ms(100);
    const char * rpm_str = "rpm: %u\n";
    char send_buf[BUF_SIZE];
    int val = 1234;
    while (sp_ctx->state != STATE_DISCONNECTED) {
      cyw43_arch_wait_for_work_until(update_deadline);
      cyw43_arch_poll();

      if (sp_ctx->ctx.client_pcb && get_absolute_time() > update_deadline) {
        update_deadline = make_timeout_time_ms(300);

        int number_of_chars = snprintf(send_buf, BUF_SIZE, rpm_str, val);
        val++;

        tcp_server_send_data(sp_ctx,
                             sp_ctx->ctx.client_pcb,
                             (const uint8_t*)send_buf,
                             number_of_chars);
      }
    }

    free(sp_ctx);

    while(true)
    {
        DEBUG_printf("\n error occured \n");
        sleep_ms(1000);
    }
}

int main() {
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

    if (cyw43_arch_init()) {
        printf("failed to initialise\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
    }
    run_tcp_server_test();
    cyw43_arch_deinit();
    return 0;
}
