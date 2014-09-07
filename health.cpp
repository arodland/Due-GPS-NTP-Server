#include "config.h"
#include "debug.h"
#include "health.h"
#include "timing.h"

static enum pll_status_t pll_status = PLL_UNLOCK;
static enum fll_status_t fll_status = FLL_UNLOCK;
static enum gps_status_t gps_status = GPS_UNLOCK;
static enum rb_status_t rb_status = RB_UNLOCK;
static enum health_status_t health_status = HEALTH_UNLOCK;
static uint32_t reftime_upper, reftime_lower;

static unsigned char gps_watchdog = 0;

void health_update();

void health_set_pll_status(enum pll_status_t status) {
  pll_status = status;
  health_update();
}

void health_set_fll_status(enum fll_status_t status) {
  fll_status = status;
  health_update();
}

void health_set_gps_status(enum gps_status_t status) {
  gps_watchdog = 0;
  gps_status = status;
  health_update();
}

void health_set_rb_status(enum rb_status_t status) {
  rb_status = status;
  health_update();
}

void health_watchdog_tick() {
  if (gps_watchdog <= 3)
    gps_watchdog ++;
  if (gps_watchdog == 3) {
    debug("GPS watchdog expired\r\n");
    gps_status = GPS_UNLOCK;
    health_update();
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

uint32_t health_get_ref_age() {
  uint32_t now_upper, now_lower, age;
  time_get_ntp(*TIMER_CLOCK, &now_upper, &now_lower, 0);
  age = now_upper - reftime_upper;
  if (now_lower < reftime_lower)
    age -= 1;
  return age;
}

/* Health state machine:
 * Initial state is UNLOCK.
 * If Rb, PLL, and GPS are all OK, state becomes OK.
 * If Rb and PLL are OK, but GPS is UNLOCK, enter HOLDOVER.
 * If Rb or PLL are UNLOCK, state is UNLOCK.
 * Can't go from UNLOCK or HOLDOVER to OK if GPS has MINOR_ALARM,
 * but if we're currently OK then a MINOR_ALARM won't make us leave.
 */
void health_update() {
  if (health_status == HEALTH_OK && gps_status == GPS_UNLOCK) {
    if (fll_status == FLL_OK) {
      health_status = HEALTH_HOLDOVER;
      pll_enter_holdover();
      debug("Health: entering HOLDOVER due to GPS unlock\r\n");
    } else {
      health_status = HEALTH_UNLOCK;
      debug("Health: entering UNLOCK due to GPS + FLL unlock\r\n");
    }
  }

  if (health_status != HEALTH_UNLOCK) {
    if (pll_status == PLL_UNLOCK) {
      health_status = HEALTH_UNLOCK;
      debug("Health: entering UNLOCK due to PLL unlock\r\n");
    }
    if (rb_status == RB_UNLOCK) {
      health_status = HEALTH_UNLOCK;
      debug("Health: entering UNLOCK due to Rb unlock\r\n");
    }
  }

  if (health_status != HEALTH_OK) {
    if (pll_status == PLL_OK && gps_status == GPS_OK && rb_status == RB_OK) {
      health_status = HEALTH_OK;
      debug("Health: all systems OK, entering OK state\r\n");
    }
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
