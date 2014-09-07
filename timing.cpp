#include <Arduino.h>
#include "config.h"
#include "debug.h"
#include "timer.h"
#include "rb.h"
#include "health.h"

static unsigned short gps_week = 0;
static uint32_t tow_sec_utc = 0;
static int startup = 1;

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
  uint32_t ns = tm * 100 + PPS_OFFSET_NS;
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
  uint32_t ntp_augmented = ntp + fudge + PPS_OFFSET_NTP;
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
  health_watchdog_tick();
}

static int pll_factor = PLL_MIN_FACTOR;
static int32_t slew_rate = 0, fll_rate = 0;
static int32_t pll_accum = 0;
static int32_t prev_pps_ns = 0;
static int32_t prev_rate = 0;
static int32_t fll_offset = 0;
static int32_t fll_history[FLL_MAX_LEN];
static uint16_t fll_history_len = 0;
static uint16_t fll_idx = 0;
static uint16_t lag = FLL_MIN_LEN;

static void pll_reset_state() {
  pll_accum = 0;
  prev_pps_ns = 0;
  prev_rate = 0;
  fll_offset = 0;
  fll_history_len = 0;
  fll_idx = 0;
  lag = FLL_MIN_LEN;
  pll_factor = PLL_MIN_FACTOR;
}

void pll_reset() {
  pll_reset_state();
  fll_rate = 0;
}

static int32_t pll_set_rate(int32_t rate) {
  int32_t rb_rate = startup ? 0 : rate;
  rb_rate = 2 * (rb_rate / 2); /* Rb granularity is 2ppt */
  rb_rate = rb_set_frequency(rb_rate);
  int32_t dds_rate = rate - rb_rate;
  int32_t timer_offs = (dds_rate + (dds_rate > 0 ? 50000 : -50000)) / 100000;
  dds_rate = 100000 * timer_offs; /* Timer granularity is 100ppb */
  timers_set_max((uint32_t) 10000000 - timer_offs);

  debug(slew_rate); debug(" PLL + "); debug(fll_rate); debug(" FLL = "); debug(rate);
  debug(" [ "); debug(rb_rate); debug(" Rb + "); debug(dds_rate); debug(" digital ]\r\n");

  return rb_rate + dds_rate;
}

void pll_run() {
  int32_t pps_ns = time_get_ns(*TIMER_CAPT_PPS, NULL) + PPS_FUDGE_NS;
  if (pps_ns > 500000000)
    pps_ns -= 1000000000;

  debug("PPS: ");
  debug(pps_ns);
  debug("\r\n");

  if (!startup && (pps_ns > 1000000 || pps_ns < -1000000)) {
    timers_jam_sync();
    pll_reset();
    return;
  }

  if (!startup) {
    fll_offset += prev_rate - 1000 * (pps_ns - prev_pps_ns);
    debug("FLLO["); debug(fll_idx); debug("]: "); debug(fll_offset);

    if (fll_history_len >= lag) {
      static int16_t fll_prev;
      if (lag == FLL_MAX_LEN) {
        fll_prev = fll_idx;
      } else {
        fll_prev = fll_idx - lag;
        if (fll_prev < 0)
          fll_prev += FLL_MAX_LEN;
        else if (fll_prev >= FLL_MAX_LEN)
          fll_prev -= FLL_MAX_LEN;
      }
      debug(" FLLOP["); debug(fll_prev); debug("]: ");
      debug(fll_history[fll_prev]);
      fll_rate = (fll_offset - fll_history[fll_prev]) / lag;
    }
    debug("\r\n");
  }

  pll_accum -= pps_ns * 1000;
  slew_rate = pll_accum / pll_factor;

  int32_t rate = slew_rate + fll_rate;
  int32_t applied_rate = pll_set_rate(rate);

  pll_accum -= (applied_rate - fll_rate) * pll_factor;

  if (startup) {
    if (slew_rate > -PLL_STARTUP_THRESHOLD && slew_rate < PLL_STARTUP_THRESHOLD) {
      startup = 0;
      pll_factor = PLL_MIN_FACTOR;
    }
  } else if (slew_rate < -PLL_STARTUP_THRESHOLD * 2 || slew_rate > PLL_STARTUP_THRESHOLD * 2) {
    pll_reset();
  }  else {
    if (pll_factor < PLL_MAX_FACTOR) {
    pll_factor ++;
    }

    fll_history[fll_idx] = fll_offset;
    fll_idx = (fll_idx + 1) % FLL_MAX_LEN;
    if (fll_history_len < FLL_MAX_LEN)
      fll_history_len ++;
    if (lag < fll_history_len && fll_idx % 2)
      lag++;
  }
  prev_pps_ns = pps_ns;
  prev_rate = rate;
  if (startup || pps_ns > PLL_HEALTHY_THRESHOLD_NS || pps_ns < -PLL_HEALTHY_THRESHOLD_NS) {
    health_set_pll_status(PLL_UNLOCK);
  } else {
    uint32_t now_upper, now_lower;
    time_get_ntp(*TIMER_CLOCK, &now_upper, &now_lower, 0);
    health_set_reftime(now_upper, now_lower);
    health_set_pll_status(PLL_OK);
  }
  if (lag < FLL_HEALTHY_THRESHOLD_SEC || fll_history_len < FLL_HEALTHY_THRESHOLD_SEC)
    health_set_fll_status(FLL_UNLOCK);
  else
    health_set_fll_status(FLL_OK);
}

void pll_enter_holdover() {
  pll_set_rate(fll_rate); /* Cancel any slew in progress but keep best known FLL value */
  pll_reset_state(); /* Everything except FLL rate will be invalid when we come out of holdover */
}

