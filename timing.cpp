#include <Arduino.h>
#include "config.h"
#include "debug.h"
#include "timer.h"

static unsigned short gps_week = 0;
static uint32_t tow_sec_utc = 0;

void time_set_date(unsigned short week, unsigned int gps_tow, short offset) {
  if ((int)gps_tow + offset < 0) {
    tow_sec_utc = gps_tow + 604800L + offset;
    gps_week = week - 1;
  } else {
    tow_sec_utc = gps_tow + offset;
    gps_week = week;
  }
}

const uint32_t PLL_OFFSET_NS = 100000000L - PLL_FUDGE_NS;

uint32_t make_ns(uint32_t tm, char *carry) {
  uint32_t ns = tm * 100 + PLL_OFFSET_NS;
  if (ns >= 1000000000L) {
    ns -= 1000000000L;
    if (carry)
      *carry = 1;
  } else if (carry)
    *carry = 0;
  return ns;
}

uint32_t time_get_ns(char *carry) {
  return make_ns(timer_get_counter(), carry);
}

uint32_t time_get_ns_capt(char *carry) {
  return make_ns(timer_get_capture(), carry);
}

inline uint32_t ntp_scale(uint32_t tm) {
  const uint32_t mult = 28147498;
  uint32_t upper = (tm >> 16) * (mult >> 16);
  uint32_t x1 = (tm >> 16) * (mult & 0xffff);
  uint32_t x2 = (tm & 0xffff) * (mult >> 16);
  uint32_t low = (tm & 0xffff) * (mult & 0xffff) + 32768;
  return (upper << 16) + x1 + x2 + (low >> 16);
}

uint32_t make_ntp(uint32_t tm, int32_t fudge, char *carry) {
  uint32_t ntp = ntp_scale(tm);
  uint32_t ntp_augmented = ntp + fudge;
  *carry = ntp_augmented < ntp ? 1 : 0;
  return ntp_augmented;
}

void time_get_ntp(uint32_t *upper, uint32_t *lower, int32_t fudge) {
  char carry;

  *upper = 2524953600UL; /* GPS epoch - NTP epoch in sec */
  *upper += gps_week * 604800UL; /* 1 week in sec */
  *upper += tow_sec_utc;

  *lower = make_ntp(timer_get_counter(), fudge, &carry);
  *upper += carry;
}

void second_int() {
  ++tow_sec_utc;
  if (tow_sec_utc >= 604800UL) {
    tow_sec_utc -= 604800UL;
    ++gps_week;
  }
}

