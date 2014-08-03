#include "config.h"
#include "debug.h"

void rb_init() {
  Rb.begin(57600);
  Rb.print("a0\r\n"); /* Disable analog frequency control */
  Rb.print("f0\r\n"); /* Zero frequency offset */
}

/* Parts per trillion -- one billion of these is 1ppm. */
void rb_set_frequency(int32_t ppt) {
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
}


