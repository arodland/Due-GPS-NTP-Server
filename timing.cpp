#include <Arduino.h>
#include "config.h"
#include "debug.h"
#include "timer.h"
#include "rb.h"
#include "health.h"
#include "monitor.h"

static unsigned short gps_week = 0;
static uint32_t tow_sec_utc = 0;
static int32_t sawtooth = 0;

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
  uint32_t ns = tm * 32 + PPS_OFFSET_NS;
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
  const uint32_t mult = 0x705f4136;
  uint32_t upper = (tm >> 16) * (mult >> 16);
  uint32_t x1 = (tm >> 16) * (mult & 0xffff);
  uint32_t x2 = (tm & 0xffff) * (mult >> 16);
  uint32_t low = (tm & 0xffff) * (mult & 0xffff);
  return tm * 137 + upper + ((x1 + x2 + (low >> 16) + 0x8032) >> 16);
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

int32_t time_get_unix() {
  int32_t ret = 315964800UL; /* GPS epoch - unix epoch in sec */
  ret += gps_week * 604800UL; /* 1 week in sec */
  ret += tow_sec_utc;
  return ret;
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
static int fll_factor = FLL_MIN_FACTOR;
static int32_t slew_rate = 0, fll_rate = FLL_START_VALUE;
static int32_t pll_accum = 0;
static int prev_valid = 0;
static int32_t prev_pps_ns = 0;
static int32_t fll_accum = 0;
static int32_t prev_slew_rate = 0;

static int pll_min_factor = PLL_MIN_FACTOR;
static int pll_max_factor = PLL_MAX_FACTOR;
static int fll_min_factor = FLL_MIN_FACTOR;
static int fll_max_factor = FLL_MAX_FACTOR;

static int jump_counter = 0;
static int uptime = 0;
static unsigned char cycle = 0;

void pll_reset_state() {
  pll_accum = 0;
  prev_valid = 0;
  prev_pps_ns = 0;
  prev_slew_rate = 0;
  fll_accum = 0;
  pll_factor = pll_min_factor;
  fll_factor = fll_min_factor;
}

void pll_reset() {
  pll_reset_state();
  fll_rate = FLL_START_VALUE;
}

static int32_t pll_set_rate(int32_t rate) {
  int32_t rb_rate = 2 * (rate / 2); /* Rb granularity is 2ppt */
  rb_rate = rb_set_frequency(rb_rate);
  int32_t dds_rate = rate - rb_rate;
  int32_t timer_offs = (dds_rate + (dds_rate > 0 ? 16000 : -16000)) / 32000;
  dds_rate = 32000 * timer_offs; /* Timer granularity is 32ppb */
  timers_set_max((uint32_t) 31250000 - timer_offs);

  debug(slew_rate); debug(" PLL + "); debug(fll_rate); debug(" FLL = "); debug(rate);
  debug(" [ "); debug(rb_rate); debug(" Rb + "); debug(dds_rate); debug(" digital ]\r\n");

  monitor_send("fll", fll_rate);
  monitor_send("freq", rate);

  return rb_rate + dds_rate;
}

void pll_run() {
  int32_t pps_ns = time_get_ns(*TIMER_CAPT_PPS, NULL) + PPS_FUDGE_NS;
  if (pps_ns > 500000000)
    pps_ns -= 1000000000;

  debug("PPS: ");
  debug(pps_ns);
  debug(" + ");
  debug(sawtooth);
//  pps_ns += sawtooth;
  debug(" = ");
  debug(pps_ns + sawtooth);
  debug("\r\n");

  monitor_send("phase", pps_ns);
  /* Ignore a jump of 1us or more by repeating the previous measurement.
   * If it persists for 3 seconds, though, allow it through.
   */
  if (prev_valid && jump_counter < 5 && (
    (pps_ns - prev_pps_ns >= 1000) || (pps_ns - prev_pps_ns <= -1000)
    )) {
    jump_counter ++;
    pps_ns = prev_pps_ns;
  } else {
    jump_counter = 0;
  }

  /* If we're more than 1ms out of whack, reset the PLL and resync
   * instead of trying to slew back into the zone.
   */
  if ((pps_ns > 1000000 || pps_ns < -1000000)) {
    timers_jam_sync();
    pll_reset();
    return;
  }


  pll_accum -= pps_ns * 1000;
  slew_rate = pll_accum / (pll_factor * PLL_SMOOTH);

  if (prev_valid) {
    fll_accum += prev_slew_rate - 1000 * (pps_ns - prev_pps_ns);
    int32_t mod_rate = fll_accum / (fll_factor * FLL_SMOOTH);
    debug("FLL: ");
    debug(fll_rate);
    fll_rate += mod_rate;
    fll_accum -= mod_rate * fll_factor;
    if (fll_rate > FLL_MAX) {
      fll_rate = FLL_MAX;
    }
    if (fll_rate < -FLL_MAX) {
      fll_rate = -FLL_MAX;
    }
    debug(" + ");
    debug(mod_rate);
    debug(" = ");
    debug(fll_rate);
    debug("\r\n");
  }

  int32_t rate = slew_rate + fll_rate;
  int32_t applied_rate = pll_set_rate(rate);

  pll_accum -= (applied_rate - fll_rate) * pll_factor;

  if (uptime < 300) {
    uptime ++;
  } else {
    cycle ++;

    if (cycle % 2 == 0 && pll_factor < pll_max_factor) {
      pll_factor ++;
    }
    if (cycle % 8 == 0 && fll_factor < fll_max_factor) {
      fll_factor ++;
    }
  }

  prev_slew_rate = applied_rate - fll_rate;
  prev_pps_ns = pps_ns;
  prev_valid = 1;

  if (pps_ns > PLL_HEALTHY_THRESHOLD_NS || pps_ns < -PLL_HEALTHY_THRESHOLD_NS) {
    health_set_pll_status(PLL_UNLOCK);
  } else {
    uint32_t now_upper, now_lower;
    time_get_ntp(*TIMER_CLOCK, &now_upper, &now_lower, 0);
    health_set_reftime(now_upper, now_lower);
    health_set_pll_status(PLL_OK);
    health_set_fll_status(FLL_OK);
    health_reset_fll_watchdog();
  }
}

void pll_enter_holdover() {
  slew_rate = 0;
  pll_set_rate(fll_rate); /* Cancel any slew in progress but keep best known FLL value */
  pll_reset_state(); /* Everything except FLL rate will be invalid when we come out of holdover */
  pll_factor = pll_max_factor / 2;
}

void time_set_sawtooth(int32_t s) {
  sawtooth = s;
}

int pll_get_factor() {
  return pll_factor;
}

void pll_set_factor(int x) {
  pll_factor = x;
}

int pll_get_min() {
  return pll_min_factor;
}

void pll_set_min(int x) {
  pll_min_factor = x;
  if (pll_factor < pll_min_factor)
    pll_factor = pll_min_factor;
}

int pll_get_max() {
  return pll_max_factor;
}

void pll_set_max(int x) {
  pll_max_factor = x;
  if (pll_factor > pll_max_factor)
    pll_factor = pll_max_factor;
}

int fll_get_factor() {
  return fll_factor;
}

void fll_set_factor(int x) {
  fll_factor = x;
}

int fll_get_min() {
  return fll_min_factor;
}

void fll_set_min(int x) {
  fll_min_factor = x;
  if (fll_factor < fll_min_factor)
    fll_factor = fll_min_factor;
}

int fll_get_max() {
  return fll_max_factor;
}

void fll_set_max(int x) {
  fll_max_factor = x;
  if (fll_factor > fll_max_factor)
    fll_factor = fll_max_factor;
}

