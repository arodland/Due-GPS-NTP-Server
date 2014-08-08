#include "config.h"
#include "debug.h"

static int32_t rb_ppt = 0;

void rb_init() {
  Rb.begin(57600);
  Rb.print("a0\r\n"); /* Disable analog frequency control */
  Rb.print("f0\r\n"); /* Zero frequency offset */
  rb_ppt = 0;
}

/* Parts per trillion -- one billion of these is 1ppm. */
void rb_set_frequency(int32_t ppt) {
  if (ppt == rb_ppt)
    return;
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
  debug("PLL: ");
  debug(ppt);
  debug("\r\n");

  rb_ppt = ppt;
}

int32_t rb_get_ppt() {
  return rb_ppt;
}

