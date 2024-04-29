#include "coater_dshot.h"

#include "hardware/clocks.h"
#include "hardware/pio.h"

#include "dshot_encoder.pio.h"

#define DEBUG_printf printf

static PIO pio = pio0;
static int pio_sm = -1;

static uint16_t dshot_get_throttlec_command(double t);

bool dshot_init(uint16_t dshot_gpio, bool enable_repeat) {
  uint pio_offset = pio_add_program(pio, &dshot_encoder_program);
  pio_sm = pio_claim_unused_sm(pio, true);

  if (pio_sm < 0) {
    pio_sm_unclaim(pio, pio_sm);
    return false;
  }

  dshot_encoder_program_init(pio, pio_sm, pio_offset, dshot_gpio, enable_repeat);
  return true;
}

void dshot_send_command(uint16_t c) {
  // Shift for telemetry bit (0)
  c = c << 1;

  // Shift and include checksum
  uint16_t checksum = (c ^ (c >> 4) ^ (c >> 8)) & 0x0F;
  c = (c << 4) | checksum;

  pio_sm_put_blocking(pio, pio_sm, c);
}

bool
set_dshot_safe(spin_coater_context_t* ctx, unsigned dshot)
{
  if (dshot < MIN_THROTTLE_COMMAND)
    dshot = MIN_THROTTLE_COMMAND;
  else if( dshot > MAX_THROTTLE_COMMAND)
    dshot = MAX_THROTTLE_COMMAND;

  ctx->dshot_throttle_val = dshot;
  DEBUG_printf("\n DSHOT set to: %lu!\n", dshot);
  dshot_send_command(ctx->dshot_throttle_val);
  return true;
}