#ifndef __TIMING_H
#define __TIMING_H

extern void time_set_date(unsigned short gps_week, unsigned int gps_tow_sec, short offset);
extern void second_int();
uint32_t time_get_ns(uint32_t tm, char *carry);
void time_get_ntp(uint32_t tm, uint32_t *upper, uint32_t *lower, int32_t fudge);
void pll_run();
extern void time_set_valid(char valid);
extern char time_get_valid();
extern void pll_reset();

#endif
