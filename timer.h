#ifndef __TIMER_H
#define __TIMER_H

volatile extern char pps_int;

extern void timer_init();

static inline uint32_t timer_get_counter() {
  return TC0->TC_CHANNEL[0].TC_CV;
}

static inline char timer_get_pending() {
  // Reading TC_SR will clear the status register, so if there really
  // *is* an event pending it will be lost. We need to either shadow
  // the register whenever we read it outside of the ISR, or count on
  // never being able to see the flags true outside of an ISR anyway.
  return 0;
  // return TC0->TC_CHANNEL[0].TC_SR & TC_SR_CPCS ? 1 : 0;
}

static inline uint32_t timer_get_capture() {
  return TC0->TC_CHANNEL[1].TC_RA;
}

extern void timers_set_max(uint32_t max);

#endif
