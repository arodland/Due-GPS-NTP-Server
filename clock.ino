#include "gps.h"

void setup() {
  SerialUSB.begin(115200);
  pmc_enable_periph_clk(ID_TC0);
  TC_Configure(TC0, 0,
    TC_CMR_TCCLKS_XC0 |
    TC_CMR_WAVSEL_UP_RC |
    TC_CMR_LDRA_RISING |
    TC_CMR_CPCTRG
  );
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS | TC_IER_LDRAS;
  TC0->TC_CHANNEL[0].TC_IDR = ~(TC_IER_CPCS | TC_IER_LDRAS);
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN;
  TC0->TC_CHANNEL[0].TC_RC = 10000000;
  NVIC_EnableIRQ(TC0_IRQn);

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

  TC0->TC_BCR = TC_BCR_SYNC;
  
  NVIC_EnableIRQ(TC1_IRQn);

  gps_init();
}

void TC0_Handler() {
  static int ct = 0;
  uint32_t status = TC0->TC_CHANNEL[0].TC_SR;
  if (status & TC_SR_CPCS) {
    SerialUSB.print("TICK: ");
    SerialUSB.println(ct++);
  }
  if (status & TC_SR_LDRAS) {
    SerialUSB.print("CAPT 0: ");
    uint32_t tm = TC0->TC_CHANNEL[0].TC_RA;
    TC0->TC_CHANNEL[0].TC_RB;
    SerialUSB.println(tm);
  }
}

void TC1_Handler() {
  uint32_t status = TC0->TC_CHANNEL[1].TC_SR;
  if (status & TC_SR_LDRAS) {
    SerialUSB.print("CAPT 1: ");
    uint32_t tm = TC0->TC_CHANNEL[1].TC_RA;
    TC0->TC_CHANNEL[1].TC_RB;
    SerialUSB.println(tm);
  }
}

void loop() {
  gps_poll();
}
