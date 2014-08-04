#include "config.h"
#include "debug.h"
#include "ethernet.h"

// TC0 Ch0 is driven by the RB, generates a 1Hz interrupt,
// and timestamps incoming Ethernet packets (rising signal on pin 2)
// then calls ether_interrupt() to handle the packet
static void timer0_setup() {
  pmc_enable_periph_clk(ID_TC0); // TC0 Ch0
  TC_Configure(TC0, 0,
    TC_CMR_TCCLKS_XC0 |          // XC0 = TCLK0 = PB26 = pin 22
    TC_CMR_WAVSEL_UP_RC |        // Count up to RC then interrupt and reset to 0
    TC_CMR_LDRA_FALLING |        // Load RA and generate interrupt on falling edge of TIOA0 (PB25 = pin 2)
    TC_CMR_CPCTRG                // Generate trigger to allow reloading of RA on RC compare
  );
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS | TC_IER_LDRAS;    // Generate interrupts on RC comapre and RA load
  TC0->TC_CHANNEL[0].TC_IDR = ~(TC_IER_CPCS | TC_IER_LDRAS);
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN; // Enable clock
  TC0->TC_CHANNEL[0].TC_RC = 10000000;      // Period = 1 second
  NVIC_EnableIRQ(TC0_IRQn);                 // Enable IRQ (TC0_Handler)
}

static void timer1_setup() {
  pmc_enable_periph_clk(ID_TC1); // TC0 Ch1
  TC_Configure(TC0, 1,
    TC_CMR_TCCLKS_XC0 |          // XC0 = TCLK0 = PB26 = pin 22
    TC_CMR_WAVSEL_UP_RC |        // Count up to RC then reset to 0
    TC_CMR_LDRA_RISING |         // Load RA and generate interrupt on rising edge of TIOA1 (PA2 = pin A7)
    TC_CMR_CPCTRG                // Generate trigger to allow reloading of RA on RC compare
  );
  TC0->TC_CHANNEL[1].TC_IER = TC_IER_LDRAS;    // Generate interrupt on RA load
  TC0->TC_CHANNEL[1].TC_IDR = ~(TC_IER_LDRAS);
  TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN;    // Enable clock
  TC0->TC_CHANNEL[1].TC_RC = 10000000;         // Period = 1 second
  NVIC_EnableIRQ(TC1_IRQn);                    // Enable IRQ (TC1_Handler)
}

static void timers_sync() {
  // Loading this register resets all three channels of TC0 to 0 and starts them
  TC0->TC_BCR = TC_BCR_SYNC;
}

void TC0_Handler() {
  static int ct = 0;
  uint32_t status = TC0->TC_CHANNEL[0].TC_SR;
  if (status & TC_SR_CPCS) { // On RC compare (1Hz)
    debug("TICK: ");
    debug(ct++); debug("\r\n");
  }
  if (status & TC_SR_LDRAS) { // On falling edge of ethernet int
    uint32_t tm = TC0->TC_CHANNEL[0].TC_RA;
    TC0->TC_CHANNEL[0].TC_RB;
    ether_interrupt(tm);
  }
}

void TC1_Handler() {
  uint32_t status = TC0->TC_CHANNEL[1].TC_SR;
  if (status & TC_SR_LDRAS) { // On rising edge of PPS
    debug("CAPT: ");
    uint32_t tm = TC0->TC_CHANNEL[1].TC_RA;
    TC0->TC_CHANNEL[1].TC_RB;
    debug(tm); debug("\r\n");
  }
}

void timer_init() {
  timer0_setup();
  timer1_setup();
  timers_sync();
}
