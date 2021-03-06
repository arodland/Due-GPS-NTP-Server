#include <Arduino.h>
#include "config.h"
#include "debug.h"
#include "timer.h"
#include "rb.h"
#include "health.h"
#include "monitor.h"
#include "ethernet.h"
#include "gps.h"

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
  uint32_t ns = ((uint64_t)tm * 1000000LL) / HZ + PPS_OFFSET_NS;
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
  const uint32_t mult_i = (1ULL<<32) / HZ;
  const uint32_t mult_f = ((1ULL<<32) - mult_i * HZ) * (1ULL<<32) / HZ;
  uint32_t upper = (tm >> 16) * (mult_f >> 16);
  uint32_t x1 = (tm >> 16) * (mult_f & 0xffff);
  uint32_t x2 = (tm & 0xffff) * (mult_f >> 16);
  uint32_t low = (tm & 0xffff) * (mult_f & 0xffff);
  return tm * mult_i + upper + ((x1 + x2 + (low >> 16) + NTP_ROUND_MAGIC) >> 16);
}

uint32_t make_ntp(uint32_t tm, int32_t fudge, char *carry) {
  uint32_t ntp = ntp_scale(tm);
  uint32_t ntp_augmented = ntp + fudge + PPS_OFFSET_NTP + NTP_FUDGE_NTP;
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
static int32_t prev_pps_filtered;
static int32_t fll_accum = 0;
static int32_t fll_2a = 0;
static int32_t fll_extra = 0;
static int32_t prev_slew_rate = 0;

static int32_t filter_carry = 0;

static int pll_min_factor = PLL_MIN_FACTOR;
static int pll_max_factor = PLL_MAX_FACTOR;
static int fll_min_factor = FLL_MIN_FACTOR;
static int fll_max_factor = FLL_MAX_FACTOR;

static int jump_counter = 0;
static int uptime = 0;
static char holdover = 0;
static unsigned char cycle = 0;

static bool pll_enabled = true;

void pll_reset_state() {
  pll_accum = 0;
  prev_valid = 0;
  prev_pps_ns = 0;
  prev_slew_rate = 0;
  fll_accum = 0;
  fll_2a = 0;
  fll_extra = 0;
  filter_carry = 0;
  prev_pps_filtered = 0;
}

void pll_reset() {
  pll_reset_state();
  fll_rate = FLL_START_VALUE;
}

static int32_t pll_set_rate(int32_t rate) {
  int32_t fll_adjusted = fll_rate + fll_extra;

  int32_t rb_rate = 2 * (rate / 2); /* Rb granularity is 2ppt */
  rb_rate = rb_set_frequency(rb_rate);
  int32_t dds_rate = rate - rb_rate;
  int32_t timer_offs = (dds_rate + (dds_rate > 0 ? 500*NSPT : -500*NSPT)) / (1000*NSPT);
  dds_rate = 1000 * NSPT * timer_offs; /* Timer granularity NSPT ppb = 1000*NSPT ppt */
  timers_set_max((uint32_t) HZ - timer_offs);

  debug(slew_rate); debug(" PLL + "); debug(fll_adjusted); debug(" FLL = "); debug(rate);
  debug(" [ "); debug(rb_rate); debug(" Rb + "); debug(dds_rate); debug(" digital ]\r\n");

  monitor_send("fll", fll_adjusted);
  monitor_send("freq", rate);

  return rb_rate + dds_rate;
}

void pll_run() {
  int32_t pps_ns;
  bool ts_from_gps = gps_get_timestamp(&pps_ns);

  if (!ts_from_gps) {
    pps_ns = time_get_ns(*TIMER_CAPT_PPS, NULL) + PPS_FUDGE_NS;
  }

  if (pps_ns > 500000000)
    pps_ns -= 1000000000;
  if (pps_ns < -500000000)
    pps_ns += 1000000000;

  debug("PPS: ");
  debug(pps_ns);
//  debug(" + ");
//  debug(sawtooth);

  monitor_send("phase_raw", pps_ns);

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

  /* If we're more than 100us out of whack, or we take a phase hit (after the
   * jump filter) of more than 10us, reset the PLL and resync instead of 
   * trying to slew back into the zone.
   */
  if (pps_ns > 100000 || pps_ns < -100000 || 
      (prev_valid && ((pps_ns - prev_pps_ns) >= 10000 || (pps_ns - prev_pps_ns) <= -10000))
    ) {
    monitor_flush();
    timers_jam_sync();
    rb_write_divisor();
    pll_reset_state();
    return;
  }

  monitor_send("phase", pps_ns);

  int32_t pps_filtered;

  if (prev_valid && uptime > 2) {
    int filter_factor = pll_factor / PPS_FILTER_DIV;
    if (filter_factor < PPS_FILTER_MIN)
      filter_factor = PPS_FILTER_MIN;
    else if (filter_factor > PPS_FILTER_MAX)
      filter_factor = PPS_FILTER_MAX;
    pps_filtered = (filter_factor - 1) * prev_pps_filtered + pps_ns + filter_carry;
    filter_carry = pps_filtered % filter_factor;
    pps_filtered /= filter_factor;
  } else {
    pps_filtered = pps_ns;
  }

  debug(" (");
  debug(pps_filtered);
  debug(")");
  monitor_send("phase_filtered", pps_filtered);

  if (ts_from_gps) {
    debug(" GPS\r\n");
  } else {
    debug("\r\n");
  }

  if (pll_enabled) {
    pll_accum -= pps_filtered * 1000;
    slew_rate = pll_accum / pll_factor;

    fll_extra = 0;

    if (prev_valid) {
      if (uptime >= 180) {
        fll_accum += prev_slew_rate - 1000 * (pps_filtered - prev_pps_filtered);
        monitor_send("fll_accum", fll_accum);

        int32_t mod_rate = 2 * fll_accum / (fll_factor * FLL_SMOOTH);
        if (mod_rate > 0) {
          mod_rate++;
        } else {
          mod_rate--;
        }
        mod_rate /= 2;

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

        fll_2a += fll_accum;
        if (fll_2a > (fll_factor * FLL_SMOOTH)) {
          fll_extra = 1;
          fll_2a -= fll_factor * FLL_SMOOTH;
        } else if (fll_2a < -(fll_factor * FLL_SMOOTH)) {
          fll_extra = -1;
          fll_2a += fll_factor * FLL_SMOOTH;
        }
      }
    }

    monitor_send("pll_factor", pll_factor);
    monitor_send("fll_factor", fll_factor);

    int32_t rate = slew_rate + fll_rate + fll_extra;
    int32_t applied_rate = pll_set_rate(rate);

    pll_accum -= (applied_rate - (fll_rate + fll_extra)) * pll_factor;

    if (uptime < 300) {
      uptime ++;
    } else {
      cycle ++;

      if (pps_filtered >= -PLL_HEALTHY_THRESHOLD_NS && pps_filtered <= PLL_HEALTHY_THRESHOLD_NS) {
        if (cycle % 2 == 0 && pll_factor < pll_max_factor) {
          pll_factor ++;
        }
        if (cycle % 3 == 0 && fll_factor < fll_max_factor) {
          fll_factor ++;
        }
      }
    }

    prev_slew_rate = applied_rate - (fll_rate + fll_extra);
  }

  prev_pps_ns = pps_ns;
  prev_pps_filtered = pps_filtered;
  if (!prev_valid) {
    prev_valid = 1;
  }

  if (pps_filtered > PLL_HEALTHY_THRESHOLD_NS || pps_filtered < -PLL_HEALTHY_THRESHOLD_NS) {
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
  holdover = 1;
  slew_rate = 0;
  pll_set_rate(fll_rate); /* Cancel any slew in progress but keep best known FLL value */
  pll_reset_state(); /* Everything except FLL rate will be invalid when we come out of holdover */
}

void pll_leave_holdover(int32_t duration) {
  debug("Leaving holdover after "); debug_int(duration); debug("s\r\n");
  while (duration > 600) {
    duration -= 600;
    if (pll_factor > pll_max_factor / 3)
      pll_factor -= pll_factor / 4;
    if (fll_factor > fll_max_factor / 3)
      fll_factor -= fll_factor / 4;
  }
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

void pll_set_enabled(bool en) {
  if (!en) {
    pll_set_rate(fll_rate);
  }

  pll_enabled = en;
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

int fll_get_coeff() {
  return fll_rate;
}

void fll_set_coeff(int x) {
  fll_rate = x;
  if (fll_rate > FLL_MAX)
    fll_rate = FLL_MAX;
  if (fll_rate < -FLL_MAX)
    fll_rate = -FLL_MAX;
}
