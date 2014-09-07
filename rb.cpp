#include "config.h"
#include "debug.h"
#include "health.h"

static int32_t rb_ppt = 0;

void rb_init() {
  Rb.begin(57600);
  Rb.print("a0\r\n"); /* Disable analog frequency control */
  Rb.print("f0\r\n"); /* Zero frequency offset */
  rb_ppt = 0;
  health_set_rb_status(RB_OK); // TODO: monitor the LOCK signal
}

/* Parts per trillion -- one billion of these is 1ppm. */
int32_t rb_set_frequency(int32_t ppt) {
  if (ppt == rb_ppt)
    return rb_ppt;
  if (ppt > rb_ppt + 2000)
    ppt = rb_ppt + 2000;
  else if (ppt < rb_ppt - 2000)
    ppt = rb_ppt - 2000;

  int32_t tens = ppt / 10;
  int32_t tenths = ppt % 10;
  if (tenths < 0)
    tenths = -tenths;
  Rb.print("f");
  Rb.print(tens);
  if (tenths) {
    Rb.print(".");
    Rb.print(tenths);
  }
  Rb.print("\r\n");

  rb_ppt = ppt;
  return rb_ppt;
}

