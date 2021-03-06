#include "config.h"
#include "debug.h"
#define HEALTH_H_DEFINE_CONSTANTS
#include "health.h"
#include "timing.h"
#include "timer.h"

static enum pll_status_t pll_status = PLL_UNLOCK;
static enum fll_status_t fll_status = FLL_UNLOCK;
static enum gps_status_t gps_status = GPS_UNLOCK;
static enum rb_status_t rb_status = RB_UNLOCK;
static enum health_status_t health_status = HEALTH_UNLOCK;
static uint32_t reftime_upper, reftime_lower;
static uint32_t entered_holdover_upper = ~0UL, entered_holdover_lower = ~0UL;

static unsigned char gps_watchdog = 0;
static uint32_t fll_watchdog = ~0UL;

void health_update();

void health_notify_change(const char *system, const char *msg[], int old_status, int new_status) {
  debug("Health: ");
  debug(system);
  debug(" changed state from ");
  debug(msg[old_status]);
  debug(" to ");
  debug(msg[new_status]);
  debug("\r\n");
}

void health_set_pll_status(enum pll_status_t status) {
  if (status != pll_status) {
    health_notify_change("PLL", pll_status_description, pll_status, status);
    pll_status = status;
    health_update();
  }
}

void health_set_fll_status(enum fll_status_t status) {
  if (status != fll_status) {
    health_notify_change("FLL", fll_status_description, fll_status, status);
    fll_status = status;
    health_update();
  }
}

void health_reset_fll_watchdog() {
  fll_watchdog = 0;
}

void health_set_gps_status(enum gps_status_t status) {
  if (status != gps_status) {
    health_notify_change("GPS", gps_status_description, gps_status, status);
    gps_status = status;
    health_update();
  }
}

void health_reset_gps_watchdog() {
  gps_watchdog = 0;
}

void health_set_rb_status(enum rb_status_t status) {
  if (status != rb_status) {
    health_notify_change("Rb", rb_status_description, rb_status, status);
    rb_status = status;
    health_update();
  }
}

void health_watchdog_tick() {
  if (gps_watchdog <= 3)
    gps_watchdog ++;
  if (gps_watchdog == 3) {
    debug("GPS watchdog expired\r\n");
    health_set_gps_status(GPS_UNLOCK);
  }
  if (fll_watchdog <= HOLDOVER_LIMIT_SEC)
    fll_watchdog ++;
  if (fll_watchdog == HOLDOVER_LIMIT_SEC) {
    debug("Holdover expired\r\n");
    health_set_fll_status(FLL_UNLOCK);
  }
}

enum health_status_t health_get_status() {
  return health_status;
}

void health_set_reftime(uint32_t upper, uint32_t lower) {
  reftime_upper = upper;
  reftime_lower = lower;
}

void health_get_reftime(uint32_t *upper, uint32_t *lower) {
  *upper = reftime_upper;
  *lower = reftime_lower;
}

static int32_t time_since(uint32_t upper, uint32_t lower) {
  uint32_t now_upper, now_lower, age;
  time_get_ntp(*TIMER_CLOCK, &now_upper, &now_lower, 0);
  age = now_upper - upper;
  if (now_lower < lower)
    age -= 1;
  return age;
}

uint32_t health_get_ref_age() {
  return time_since(reftime_upper, reftime_lower);
}

/* Health state machine:
 * Initial state is UNLOCK.
 * If Rb, PLL, and GPS are all OK, state becomes OK.
 * If Rb and PLL are OK, but GPS is UNLOCK, enter HOLDOVER.
 * If in HOLDOVER and FLL solution becomes too old, enter UNLOCK.
 * If Rb or PLL are UNLOCK, state is UNLOCK.
 * Can't go from UNLOCK or HOLDOVER to OK if GPS has MINOR_ALARM,
 * but if we're currently OK then a MINOR_ALARM won't make us leave.
 */
void health_update() {
  health_status_t new_status = health_status;

  if (health_status == HEALTH_OK && gps_status == GPS_UNLOCK) {
    if (fll_status == FLL_OK) {
      new_status = HEALTH_HOLDOVER;
    } else {
      new_status = HEALTH_UNLOCK;
    }
  }

  if (health_status == HEALTH_HOLDOVER) {
    if (fll_status == FLL_UNLOCK) {
      new_status = HEALTH_UNLOCK;
    }
  }

  if (health_status != HEALTH_UNLOCK) {
    if (pll_status == PLL_UNLOCK) {
      new_status = HEALTH_UNLOCK;
    }
    if (rb_status == RB_UNLOCK) {
      new_status = HEALTH_UNLOCK;
    }
  }

  if (health_status != HEALTH_OK) {
    if (pll_status == PLL_OK && gps_status == GPS_OK && rb_status == RB_OK) {
      new_status = HEALTH_OK;
    }
  }

  if (new_status != health_status) {
    health_notify_change("Health", health_status_description, health_status, new_status);
    if (new_status == HEALTH_OK) {
      if (entered_holdover_lower != ~0UL || entered_holdover_upper != ~0UL)
        pll_leave_holdover(time_since(entered_holdover_upper, entered_holdover_lower));
      pps_output_enable();
    } else if (new_status == HEALTH_HOLDOVER) {
      entered_holdover_upper = reftime_upper;
      entered_holdover_lower = reftime_lower;
      pll_enter_holdover();
    } else {
      pps_output_disable();
    }
    health_status = new_status;
  }
}

char health_should_run_pll() {
  if (health_status == HEALTH_OK)
    return 1;
  if (health_status == HEALTH_HOLDOVER)
    return 0;
  if (gps_status == GPS_UNLOCK || rb_status == RB_UNLOCK)
    return 0;
  return 1;
}

void health_print_status() {
  if (health_status != HEALTH_OK) {
    debug("Health: ");
    debug(health_status_description[health_status]);
    if (rb_status != RB_OK) {
      debug(" Rb: ");
      debug(rb_status_description[rb_status]);
    }
    if (gps_status != GPS_OK) {
      debug(" GPS: ");
      debug(gps_status_description[gps_status]);
    }
    if (fll_status != FLL_OK) {
      debug(" FLL: ");
      debug(fll_status_description[fll_status]);
    }
    if (pll_status != PLL_OK) {
      debug(" PLL: ");
      debug(pll_status_description[pll_status]);
    }
    debug("\r\n");
  }
}
