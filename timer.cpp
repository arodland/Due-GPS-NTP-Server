#include "config.h"
#include "debug.h"
#include "ethernet.h"

static void timer0_setup() {
  pmc_enable_periph_clk(ID_TC0);
  TC_Configure(TC0, 0,
    TC_CMR_TCCLKS_XC0 |
    TC_CMR_WAVSEL_UP_RC |
    TC_CMR_LDRA_FALLING |
    TC_CMR_CPCTRG
  );
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS | TC_IER_LDRAS;
  TC0->TC_CHANNEL[0].TC_IDR = ~(TC_IER_CPCS | TC_IER_LDRAS);
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN;
  TC0->TC_CHANNEL[0].TC_RC = 10000000;
  NVIC_EnableIRQ(TC0_IRQn);
}

static void timer1_setup() {
  pmc_enable_periph_clk(ID_TC1);
  TC_Configure(TC0, 1,
    TC_CMR_TCCLKS_XC0 |
    TC_CMR_WAVSEL_UP_RC |
    TC_CMR_LDRA_RISING |
    TC_CMR_CPCTRG
  );
  TC0->TC_CHANNEL[1].TC_IER = TC_IER_LDRAS;
  TC0->TC_CHANNEL[1].TC_IDR = ~(TC_IER_LDRAS);
  TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN;
  TC0->TC_CHANNEL[1].TC_RC = 10000000;
  NVIC_EnableIRQ(TC1_IRQn);
}

static void timers_sync() {
  TC0->TC_BCR = TC_BCR_SYNC;
}

void TC0_Handler() {
  static int ct = 0;
  uint32_t status = TC0->TC_CHANNEL[0].TC_SR;
  if (status & TC_SR_CPCS) {
    debug("TICK: ");
    debug(ct++); debug("\r\n");
  }
  if (status & TC_SR_LDRAS) {
    uint32_t tm = TC0->TC_CHANNEL[0].TC_RA;
    TC0->TC_CHANNEL[0].TC_RB;
    ether_interrupt(tm);
  }
}

void TC1_Handler() {
  static int first = 1;
  uint32_t status = TC0->TC_CHANNEL[1].TC_SR;
  if (status & TC_SR_LDRAS) {
    debug("CAPT: ");
    uint32_t tm = TC0->TC_CHANNEL[1].TC_RA;
    TC0->TC_CHANNEL[1].TC_RB;
    debug(tm); debug("\r\n");
    if (first) {
      timers_sync();
      first = 0;
    }
  }
}

void timer_init() {
  timer0_setup();
  timer1_setup();
  timers_sync();
}
