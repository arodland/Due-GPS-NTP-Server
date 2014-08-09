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
  static int startup = 1;
  static int startup_timer = 0;
  static int pll_factor = 10;
  static int32_t prev_pps_ns = 0;
  static int32_t prev_rate = 0;
  static int32_t fll_offset = 0;
  static int32_t fll_history[1000];
  static uint16_t fll_history_len = 0;
  static uint16_t fll_idx = 0;

  int32_t pps_ns = time_get_ns(*TIMER_CAPT_PPS, NULL);
  if (pps_ns > 500000000)
    pps_ns -= 1000000000;

  debug("PPS: ");
  debug(pps_ns);
  debug("\r\n");

  int32_t fll_rate = 0;

  if (!startup) {
    fll_offset += prev_rate - 1000 * (pps_ns - prev_pps_ns);
    debug("FLLO["); debug(fll_idx); debug("]: "); debug(fll_offset);

    if (fll_history_len >= 100) {
      static int16_t lag, fll_prev;
      if (fll_history_len == 1000) {
        lag = 1000;
        fll_prev = fll_idx;
      } else {
        lag = 100;
        fll_prev = fll_idx - 100;
      }
      debug(" FLLOP["); debug(fll_prev); debug("]: ");
      debug(fll_history[fll_prev]);
      fll_rate = (fll_offset - fll_history[fll_prev]) / lag;
    }
    debug("\r\n");
  }

  int32_t slew_rate;
  if (pll_factor == 1000)
    slew_rate = -pps_ns;
  else
    slew_rate = -(pps_ns / pll_factor) * 1000;

  int32_t rate = slew_rate + fll_rate;

/*  if (rate > 1000000)
    rate = 1000000;
  else if (rate < -1000000)
    rate = -1000000; */
  rb_set_frequency(startup ? 0 : rate);
  int32_t rb_rate = rb_get_ppt();
  int32_t dds_rate = rate - rb_rate;
  timers_set_max((uint32_t)(10000000 - dds_rate / 100000));

  debug(slew_rate); debug(" PLL + "); debug(fll_rate); debug(" FLL = "); debug(rate);
  debug(" [ "); debug(rb_rate); debug(" Rb + "); debug(dds_rate); debug(" digital ]\r\n");

  if (!startup) {
    fll_history[fll_idx] = fll_offset;
    fll_idx = (fll_idx + 1) % 1000;
    if (fll_history_len < 1000)
      fll_history_len ++;
  }

  if (startup) {
    startup_timer ++;
    if (startup_timer == 60) {
      startup = 0;
      pll_factor = 100;
    }
  } else if (pll_factor < 1000) {
    pll_factor ++;
  }
  prev_pps_ns = pps_ns;
  prev_rate = rate;
}

