#ifndef __HEALTH_H
#define __HEALTH_H

enum pll_status_t {
  PLL_UNLOCK,
  PLL_OK
};

enum gps_status_t {
  GPS_UNLOCK,
  GPS_MINOR_ALARM,
  GPS_OK
};

enum rb_status_t {
  RB_UNLOCK,
  RB_OK
};

enum health_status_t {
  HEALTH_UNLOCK,
  HEALTH_HOLDOVER,
  HEALTH_OK
};

void health_set_pll_status(enum pll_status_t status);
void health_set_gps_status(enum gps_status_t status);
void health_set_rb_status(enum rb_status_t status);

void health_watchdog_tick();

enum health_status_t health_get_status();

void health_set_reftime(uint32_t upper, uint32_t lower);
void health_get_reftime(uint32_t *upper, uint32_t *lower);
uint32_t health_get_ref_age();

#endif
