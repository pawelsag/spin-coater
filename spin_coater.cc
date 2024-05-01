/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Output PWM signals on pins 0 and 1

#include "hardware/timer.h"
#include "lwip/netif.h"
#include <chrono>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "lwip/pbuf.h"
#include "lwip/tcp.h"

#include "spin_coater.h"
#if SPIN_COATER_PWM_ENABLED
#include "coater_pwm.h"
#elif SPIN_COATER_DSHOT_ENABLED
#include "coater_dshot.h"
#endif

/* Since distance between wholes is not ideally equal
   we use count_of_measurements variable to omit in between wholes and meassure time
   after full cycle */
constexpr uint64_t number_of_wholes = 2;
uint32_t count_of_measurements;

constexpr uint32_t microseconds_in_minute = 60000000;
uint32_t last_time;
uint32_t current_rpm;

#define  RPM_UPDATE_DEFAULT_DELAY 300
#define  MAX_RPM_VALUE 6000

static err_t
tcp_server_send_data(spin_coater_context_t* ctx,
                     struct tcp_pcb* tpcb,
                     const uint8_t* data,
                     u16_t size);

template<typename Callable>
static void
command_handler(spin_coater_context_t* ctx,
                uint8_t* buffer,
                Callable& feedback);


void
gpio_callback(uint gpio, uint32_t events)
{
  if (count_of_measurements == 0) {
    last_time = time_us_32();
    count_of_measurements++;
    return;
  }

  if (count_of_measurements < number_of_wholes) {
    count_of_measurements++;
    return;
  }

  uint32_t current_time_us;
  uint32_t time_elapsed_us;
  current_time_us = time_us_32();
  time_elapsed_us = current_time_us - last_time;
  current_rpm = (microseconds_in_minute / time_elapsed_us);
  count_of_measurements = 0;
}

static err_t
tcp_server_close(void* arg)
{
  spin_coater_context_t* sp_ctx = (spin_coater_context_t*)arg;
  tcp_server_context_t* ctx = &sp_ctx->ctx;
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
  spin_coater_context_t* ctx = (spin_coater_context_t*)ctx_;

  return ERR_OK;
}

static err_t
tcp_server_send_data(spin_coater_context_t* ctx,
                     struct tcp_pcb* tpcb,
                     const uint8_t* data,
                     u16_t size)
{
  return tcp_write(tpcb, data, size, TCP_WRITE_FLAG_COPY);
}

static void
tcp_command_handler(spin_coater_context_t* sp_ctx, struct tcp_pcb* tpcb)
{
  auto feedback_send = [&sp_ctx, &tpcb](const char* msg, const size_t msg_len) {
    tcp_server_send_data(sp_ctx, tpcb, (const uint8_t*)msg, msg_len);
  };

  command_handler(sp_ctx, sp_ctx->ctx.recv_buffer, feedback_send);
}

int64_t
timer_spin_callback(alarm_id_t id, void* user_data)
{
  spin_coater_context_t* ctx = (spin_coater_context_t*)user_data;

  ctx->spin_state = SPIN_SMOOTH_STOP_REQUESTED;
  const char* msg = "spin_stopped\n";
  tcp_server_send_data(
    ctx, ctx->ctx.client_pcb, (const uint8_t*)msg, strlen(msg));

  return 0;
}

template<typename Callable>
void send_help(Callable& feedback)
{
 const char *help = "Spin coater avaliable commands: \n"
 "\"help\" - send this help msg\n"
#if SPIN_COATER_PWM_ENABLED
 "\"pwm: <val>\" - start spinning with given pwm value, can be <49, 98> \n"
#elif SPIN_COATER_DSHOT_ENABLED
 "\"dshot: <val>\" - start spinning with given dshot value, can be <48, 2048> \n"
#endif
 "\"spin_start:  <time> <rpm>\" - set spinner in automatic mode. Spinner will spin for given time and adjust the speed to requested RPM value. <rpm> can be up to 6000\n"
 "\"spin_stop\" - stop spinner\n"
 "\"speedup_delay: <delay>\" - set delay between dshot increase value during RPM speed up\n"
 "\"slowdown_delay: <delay>\" - set delay between dshot decrease value during RPM slow down\n";

 feedback(help, strlen(help));
}

template<typename Callable>
static void
command_handler(spin_coater_context_t* ctx, uint8_t* buffer, Callable& feedback)
{
  unsigned arg, arg2;
  char str_arg[BUF_SIZE];

  if (buffer[0] == '\n')
    return;

#if SPIN_COATER_PWM_ENABLED
  if (sscanf((const char*)buffer, "pwm: %u", &arg) == 1) {
    const bool res = set_pwm_safe(ctx, arg);
    if (!res) {
      DEBUG_printf("Chaning PWM duty failed\n");
      return;
    }
    if (arg == PWM_IDLE_DUTY) {
      ctx->spin_state = SPIN_IDLE;
      cancel_alarm(ctx->timer_id);
      const char* msg = "spin_stopped\n";
      feedback(msg, strlen(msg));
    } else {
      ctx->spin_state = SPIN_STARTED_WITH_FORCE_VALUE;
    }

    char msg[BUF_SIZE];
    const size_t msg_len =
      snprintf(msg, sizeof(msg), "pwm: %u\n", ctx->pwm_duty);
    feedback(msg, msg_len);
#elif SPIN_COATER_DSHOT_ENABLED
  if (sscanf((const char*)buffer, "dshot: %u", &arg) == 1) {
    const bool res = set_dshot_safe(ctx, arg);
    if (!res) {
      DEBUG_printf("Chaning dshot value failed\n");
      return;
    }

    if (arg == MIN_THROTTLE_COMMAND) {
      ctx->spin_state = SPIN_IDLE;
      cancel_alarm(ctx->timer_id);
      const char* msg = "spin_stopped\n";
      feedback(msg, strlen(msg));
    } else {
      ctx->spin_state = SPIN_STARTED_WITH_FORCE_VALUE;
    }

    char msg[BUF_SIZE];
    const size_t msg_len =
      snprintf(msg, sizeof(msg), "dshot: %u\n", ctx->dshot_throttle_val);
    feedback(msg, msg_len);
#endif
  } else if (sscanf((const char*)buffer, "spin_start: %u %u", &arg, &arg2) ==
             2) {
    if(arg2 > MAX_RPM_VALUE)
    {
      DEBUG_printf("RPM value to high. Max RPM can be %d\n", MAX_RPM_VALUE);
      return;
    }
    ctx->set_rpm = arg2;
#if SPIN_COATER_PWM_ENABLED
    ctx->pwm_duty = PWM_HEAVY_LOADED_IDLE_DUTY;
#elif SPIN_COATER_DSHOT_ENABLED
    ctx->dshot_throttle_val = DSHOT_HEAVY_LOADED_IDLE_DUTY;
#endif
    ctx->spin_state = SPIN_STARTED_WITH_TIMER;
    ctx->timer_id =
      add_alarm_in_ms(std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::seconds(arg))
                        .count(),
                      timer_spin_callback,
                      ctx,
                      false);

    const char* msg = "spin_started\n";
    feedback(msg, strlen(msg));
    DEBUG_printf("Spin activated");
  } else if (strncmp((const char*)buffer, "spin_stop\n", 10) == 0) {
#if SPIN_COATER_PWM_ENABLED
    const bool res = set_pwm_safe(ctx, PWM_IDLE_DUTY);
    if (!res) {
      DEBUG_printf("Chaning PWM duty failed\n");
      return;
    }
#elif SPIN_COATER_DSHOT_ENABLED
  const bool res = set_dshot_safe(ctx, MIN_THROTTLE_COMMAND);
  if (!res) {
    DEBUG_printf("Chaning dshot value failed\n");
    return;
  }
#endif
    ctx->spin_state = SPIN_SMOOTH_STOP_REQUESTED;
    cancel_alarm(ctx->timer_id);
    const char* msg = "spin_stopped\n";
    feedback(msg, strlen(msg));
  } else if(sscanf((const char*)buffer, "speedup_delay: %u", &arg) == 1) {
    ctx->rpm_speedup_update_delay = arg;
  } else if(sscanf((const char*)buffer, "slowdown_delay: %u", &arg) == 1) {
    ctx->rpm_slowdown_update_delay = arg;
  }else if (strncmp((const char*)buffer, "help", 4)== 0){
    send_help(feedback);
  }
}

err_t
tcp_server_recv(void* ctx_, struct tcp_pcb* tpcb, struct pbuf* p, err_t err)
{
  spin_coater_context_t* sp_ctx = (spin_coater_context_t*)ctx_;
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
  tcp_command_handler(sp_ctx, tpcb);

  ctx->recv_len = 0;

  return ERR_OK;
}

static void
tcp_server_err(void* ctx_, err_t err)
{
  spin_coater_context_t* sp_ctx = (spin_coater_context_t*)ctx_;

  if (err != ERR_ABRT) {
    DEBUG_printf("tcp_client_err_fn %d\n", err);
    sp_ctx->connection_state = STATE_DISCONNECTED;
    tcp_server_close(sp_ctx);
  }
}

static err_t
tcp_server_accept(void* ctx_, struct tcp_pcb* client_pcb, err_t err)
{
  spin_coater_context_t* sp_ctx = (spin_coater_context_t*)ctx_;

  if (err != ERR_OK || client_pcb == NULL) {
    DEBUG_printf("Failure in accept\n");
    sp_ctx->connection_state = STATE_DISCONNECTED;
    tcp_server_close(sp_ctx);
    return ERR_VAL;
  }

  DEBUG_printf("Client connected\n");

  sp_ctx->ctx.client_pcb = client_pcb;
  tcp_arg(client_pcb, sp_ctx);
  tcp_sent(client_pcb, tcp_server_sent);
  tcp_recv(client_pcb, tcp_server_recv);
  tcp_err(client_pcb, tcp_server_err);

  sp_ctx->connection_state = STATE_CONNECTED;

  auto feedback_send = [&sp_ctx, client_pcb](const char* msg, const size_t msg_len) {
    tcp_server_send_data(sp_ctx, client_pcb, (const uint8_t*)msg, msg_len);
  };
  send_help(feedback_send);
  return ERR_OK;
}

static bool
tcp_server_open(void* ctx_)
{
  spin_coater_context_t* sp_ctx = (spin_coater_context_t*)ctx_;
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

  sp_ctx->connection_state = STATE_LISTENING;
  tcp_arg(ctx->server_pcb, sp_ctx);
  tcp_accept(ctx->server_pcb, tcp_server_accept);

  return true;
}

#if SPIN_COATER_PWM_ENABLED
absolute_time_t do_pwm_smooth_transition(spin_coater_context_t* sp_ctx)
{
  int direction = current_rpm < sp_ctx->set_rpm ? 1 : -1;

  sp_ctx->pwm_duty += direction;
  set_pwm_safe(sp_ctx, sp_ctx->pwm_duty);

  return make_timeout_time_ms(100);
}
#elif SPIN_COATER_DSHOT_ENABLED

int scale_dshot_value_when_speeding(spin_coater_context_t* sp_ctx, int rpm_left, int bias){
    return (50.0 / sp_ctx->set_rpm) * rpm_left + bias;
}

int scale_dshot_value_when_stopping(spin_coater_context_t* sp_ctx, int rpm_left){
    return 50.0 - (50.0 / sp_ctx->set_rpm) * rpm_left;
}

absolute_time_t do_dshot_smooth_transition(spin_coater_context_t* sp_ctx)
{
  int direction = current_rpm < sp_ctx->set_rpm ? 1 : -1;
  int rpm_diff = sp_ctx->set_rpm - current_rpm;
  absolute_time_t next_delay;
  if(SPIN_STARTED_WITH_TIMER == sp_ctx->spin_state){
    if(abs(rpm_diff) > 100)
      sp_ctx->dshot_throttle_val += scale_dshot_value_when_speeding(sp_ctx, abs(rpm_diff), 10*(current_rpm/double(sp_ctx->set_rpm)))*direction;
    else if (abs(rpm_diff) > 10 )
      sp_ctx->dshot_throttle_val += 1*direction;
    next_delay = make_timeout_time_ms(sp_ctx->rpm_speedup_update_delay);
  }
  else if(SPIN_SMOOTH_STOP_REQUESTED == sp_ctx->spin_state) {
    sp_ctx->dshot_throttle_val -= scale_dshot_value_when_stopping(sp_ctx, abs(rpm_diff));
      if(abs(rpm_diff) > (sp_ctx->set_rpm - 250) || sp_ctx->dshot_throttle_val < MIN_THROTTLE_COMMAND) {
        sp_ctx->dshot_throttle_val = MIN_THROTTLE_COMMAND;
        sp_ctx->spin_state = SPIN_IDLE;
      }
    next_delay = make_timeout_time_ms(sp_ctx->rpm_slowdown_update_delay);
  }else {
   next_delay = make_timeout_time_ms(1000); 
  }
  set_dshot_safe(sp_ctx, sp_ctx->dshot_throttle_val);
  return next_delay;
}
#endif

void
run_tcp_server_test(spin_coater_context_t* sp_ctx)
{
  if (!tcp_server_open(sp_ctx)) {
    return;
  }

  absolute_time_t update_deadline = make_timeout_time_ms(100);
  absolute_time_t rpm_notify_deadline = make_timeout_time_ms(1);
  const char* rpm_str = "rpm: %u\n";
  char send_buf[BUF_SIZE];

  while (sp_ctx->connection_state != STATE_DISCONNECTED) {
    cyw43_arch_wait_for_work_until(update_deadline);
    cyw43_arch_poll();

    if (sp_ctx->ctx.client_pcb && get_absolute_time() > rpm_notify_deadline && sp_ctx->spin_state != SPIN_IDLE) {
      int number_of_chars =
        snprintf(send_buf,
                 BUF_SIZE,
                 rpm_str,
                 current_rpm);

      tcp_server_send_data(sp_ctx,
                           sp_ctx->ctx.client_pcb,
                           (const uint8_t*)send_buf,
                           number_of_chars);

      rpm_notify_deadline = make_timeout_time_ms(300);
    }

    if ( get_absolute_time() > update_deadline ) {
      if ((sp_ctx->spin_state == SPIN_STARTED_WITH_TIMER) || (sp_ctx->spin_state == SPIN_SMOOTH_STOP_REQUESTED)) {
#if SPIN_COATER_PWM_ENABLED
        update_deadline = do_pwm_smooth_transition(sp_ctx);
#elif SPIN_COATER_DSHOT_ENABLED
        update_deadline = do_dshot_smooth_transition(sp_ctx);
#endif
      } else {
        update_deadline = make_timeout_time_ms(200);
      }
    }
  }

  free(sp_ctx);

  while (true) {
    DEBUG_printf("\n error occured \n");
    sleep_ms(1000);
  }
}

int
main()
{
  stdio_init_all();

  gpio_set_irq_enabled_with_callback(
    22, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

#if SPIN_COATER_PWM_ENABLED
  // Tell GPIO 0 and 1 they are allocated to the PWM
  gpio_set_function(SPIN_COATER_LOGIC_GP_NUM, GPIO_FUNC_PWM);

  // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
  uint slice_num = pwm_gpio_to_slice_num(SPIN_COATER_LOGIC_GP_NUM);

  // Set channel A output high for one cycle before dropping
  pwm_set_freq_duty(slice_num, PWM_CHAN_A, PWM_INIT_FREQ, PWM_IDLE_DUTY);

  // Set initial B output high for three cycles before dropping
  // Set the PWM running
  pwm_set_enabled(slice_num, true);
#elif SPIN_COATER_DSHOT_ENABLED
    dshot_init(SPIN_COATER_LOGIC_GP_NUM);
    dshot_send_command(MIN_THROTTLE_COMMAND);
#endif

  if (cyw43_arch_init()) {
    printf("failed to initialise\n");
    return 1;
  }

  cyw43_arch_enable_sta_mode();

  printf("Connecting to Wi-Fi... SSID:%s \n", WIFI_SSID);
  if (cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
    printf("failed to connect to SSID: %s.\n", WIFI_SSID);
    return 1;
  } else {
    printf("Connected.\n");
  }

  spin_coater_context_t* sp_ctx =
    (spin_coater_context_t*)calloc(1, sizeof(spin_coater_context_t));
  if (!sp_ctx) {
    DEBUG_printf("failed to allocate state\n");
    return 1;
  }

#if SPIN_COATER_PWM_ENABLED
  sp_ctx->pwm_slice_num = slice_num;
  sp_ctx->pwm_duty = PWM_IDLE_DUTY;
#elif SPIN_COATER_DSHOT_ENABLED
  sp_ctx->dshot_throttle_val = MIN_THROTTLE_COMMAND;
#endif
  sp_ctx->set_rpm = 0;
  sp_ctx->spin_state = SPIN_IDLE;
  sp_ctx->rpm_speedup_update_delay = RPM_UPDATE_DEFAULT_DELAY;
  sp_ctx->rpm_slowdown_update_delay = RPM_UPDATE_DEFAULT_DELAY;

  run_tcp_server_test(sp_ctx);
  cyw43_arch_deinit();
  return 0;
}
