#ifndef __HEALTH_H
#define __HEALTH_H

enum pll_status_t {
  PLL_UNLOCK,
  PLL_OK
};

#ifdef HEALTH_H_DEFINE_CONSTANTS
char *pll_status_description[] = {
  [PLL_UNLOCK] = "UNLOCK",
  [PLL_OK]     = "OK"
};
#endif

enum fll_status_t {
  FLL_UNLOCK,
  FLL_OK
};

#ifdef HEALTH_H_DEFINE_CONSTANTS
char *fll_status_description[] = {
  [FLL_UNLOCK] = "UNLOCK",
  [FLL_OK]     = "OK"
};
#endif

enum gps_status_t {
  GPS_UNLOCK,
  GPS_MINOR_ALARM,
  GPS_OK
};

#ifdef HEALTH_H_DEFINE_CONSTANTS
char *gps_status_description[] = {
  [GPS_UNLOCK]      = "UNLOCK",
  [GPS_MINOR_ALARM] = "MINOR ALARM",
  [GPS_OK]          = "OK"
};
#endif

enum rb_status_t {
  RB_UNLOCK,
  RB_OK
};

#ifdef HEALTH_H_DEFINE_CONSTANTS
char *rb_status_description[] = {
  [RB_UNLOCK] = "UNLOCK",
  [RB_OK]     = "OK"
};
#endif

enum health_status_t {
  HEALTH_UNLOCK,
  HEALTH_HOLDOVER,
  HEALTH_OK
};

#ifdef HEALTH_H_DEFINE_CONSTANTS
char *health_status_description[] = {
  [HEALTH_UNLOCK]   = "UNLOCK",
  [HEALTH_HOLDOVER] = "HOLDOVER",
  [HEALTH_OK]       = "OK"
};
#endif

void health_set_pll_status(enum pll_status_t status);
void health_set_fll_status(enum fll_status_t status);
void health_set_gps_status(enum gps_status_t status);
void health_set_rb_status(enum rb_status_t status);

void health_watchdog_tick();

enum health_status_t health_get_status();
char health_should_run_pll();

void health_set_reftime(uint32_t upper, uint32_t lower);
void health_get_reftime(uint32_t *upper, uint32_t *lower);
uint32_t health_get_ref_age();

#endif
