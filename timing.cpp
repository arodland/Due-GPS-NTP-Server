#include <Arduino.h>
#include "config.h"
#include "debug.h"
#include "timer.h"
#include "rb.h"

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

uint32_t make_ns(uint32_t tm, char *carry) {
  uint32_t ns = tm * 100;
  if (ns >= 1000000000L) {
    ns -= 1000000000L;
    if (carry)
      *carry = 1;
  } else if (carry)
    *carry = 0;
  return ns;
}

uint32_t time_get_ns(uint32_t tm, char *carry) {
  return make_ns(tm, carry);
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
  if (carry)
    *carry = ntp_augmented < ntp ? 1 : 0;
  return ntp_augmented;
}

void time_get_ntp(uint32_t tm, uint32_t *upper, uint32_t *lower, int32_t fudge) {
  char carry;

  *upper = 2524953600UL; /* GPS epoch - NTP epoch in sec */
  *upper += gps_week * 604800UL; /* 1 week in sec */
  *upper += tow_sec_utc;

  *lower = make_ntp(tm, fudge, &carry);
  *upper += carry;
}

void second_int() {
  ++tow_sec_utc;
  if (tow_sec_utc >= 604800UL) {
    tow_sec_utc -= 604800UL;
    ++gps_week;
  }
}

void pll_run() {
  static int pll_factor = 30;

  int32_t pps_ns = time_get_ns(*TIMER_CAPT_PPS, NULL);
  if (pps_ns > 500000000)
    pps_ns -= 1000000000;

  debug("PPS: ");
  debug(pps_ns);
  debug("\r\n");

  int32_t rate;
  if (pll_factor == 1000)
    rate = -pps_ns;
  else
    rate = -(pps_ns / pll_factor) * 1000;
/*  if (rate > 1000000)
    rate = 1000000;
  else if (rate < -1000000)
    rate = -1000000; */
  rb_set_frequency(rate);
  int32_t rb_ppt = rb_get_ppt();
  int32_t error = rate - rb_ppt;
  timers_set_max((uint32_t)(10000000 - error / 100000));

  if (pll_factor < 1000)
    pll_factor++;
}

