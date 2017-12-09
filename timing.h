#ifndef __TIMING_H
#define __TIMING_H

extern void time_set_date(unsigned short gps_week, unsigned int gps_tow_sec, short offset);
extern void second_int();
uint32_t time_get_ns(uint32_t tm, char *carry);
void time_get_ntp(uint32_t tm, uint32_t *upper, uint32_t *lower, int32_t fudge);
extern int32_t time_get_unix();
void pll_run();
extern void time_set_valid(char valid);
extern char time_get_valid();
extern void pll_reset();
extern void pll_reset_state();
extern void pll_enter_holdover();
extern void pll_leave_holdover(int32_t duration);
extern void time_set_sawtooth(int32_t s);


extern int pll_get_factor();
extern void pll_set_factor(int);
extern int pll_get_min();
extern void pll_set_min(int);
extern int pll_get_max();
extern void pll_set_max(int);

extern int fll_get_factor();
extern void fll_set_factor(int);
extern int fll_get_min();
extern void fll_set_min(int);
extern int fll_get_max();
extern void fll_set_max(int);
extern int fll_get_coeff();
extern void fll_set_coeff(int);
#endif
