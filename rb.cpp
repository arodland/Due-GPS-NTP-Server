#include "config.h"
#include "debug.h"
#include "health.h"

static int32_t rb_ppt = 0;

void rb_update_health();

void rb_init() {
  Rb.begin(57600);
  Rb.print("a0\r\n"); /* Disable analog frequency control */
  Rb.print("f0\r\n"); /* Zero frequency offset */
  rb_ppt = 0;
  pinMode(53, INPUT);
  attachInterrupt(53, rb_update_health, CHANGE);
  rb_update_health();
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

  String buf = "f";
  buf += String(tens, DEC);
  if (tenths) {
    buf += ".";
    buf += String(tenths, DEC);
  }
  buf += "\r\n";
  Rb.print(buf);

  rb_ppt = ppt;
  return rb_ppt;
}

void rb_update_health() {
  int lock = digitalRead(53);
  if (lock) { /* High = unlocked */
    health_set_rb_status(RB_UNLOCK);
  } else {
    health_set_rb_status(RB_OK);
  }
}
