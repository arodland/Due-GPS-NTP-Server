#include <Arduino.h>
#include "debug.h"

void time_set_date(unsigned short gps_week, unsigned int gps_tow_sec, short offset) {
  debug("Set date: week "); debug(gps_week);
  debug(" TOW "); debug(gps_tow_sec);
  debug(" GPS-UTC "); debug(offset);
  debug("\r\n");
}
