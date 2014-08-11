#include "config.h"
#include "debug.h"
#include "ethernet.h"
#include "timing.h"

volatile char pps_int = 0;


static void timer1_setup() {
  pmc_enable_periph_clk(ID_TC1); // TC0 Ch1
  TC_Configure(TC0, 1,
    TC_CMR_TCCLKS_XC0 |          // XC0 = TCLK0 = PB26 = pin 22
    TC_CMR_WAVSEL_UP_RC |        // Count up to RC then reset to 0
    TC_CMR_LDRA_RISING |         // Load RA and generate interrupt on rising edge of TIOA1 (PA2 = pin A7)
    TC_CMR_CPCTRG                // Generate trigger to allow reloading of RA on RC compare
  );
  TC0->TC_CHANNEL[1].TC_IER = TC_IER_LDRAS | TC_IER_CPCS;    // Generate interrupt on RA load and 1Hz
  TC0->TC_CHANNEL[1].TC_IDR = ~(TC_IER_LDRAS | TC_IER_CPCS);
  TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN;    // Enable clock
  TC0->TC_CHANNEL[1].TC_RC = 10000000;         // Period = 1 second
  NVIC_EnableIRQ(TC1_IRQn);                    // Enable IRQ (TC1_Handler)
}

static void timer0_setup() {
  pmc_enable_periph_clk(ID_TC0);
  TC_Configure(TC0, 0,
    TC_CMR_TCCLKS_XC0 |          // XC0 = TCLK0 = PB26 = pin 22
    TC_CMR_WAVSEL_UP_RC |        // Count up to RC then reset to 0
    TC_CMR_WAVE |                // Generate output waveform
    TC_CMR_EEVT_XC1 |            // Let TIOB be an output
    TC_CMR_BCPB_SET |            // Rising on RB compare (TIOB = PB27 = LED pin 13)
    TC_CMR_BCPC_CLEAR            // Falling on RC compare
  );
  TC0->TC_CHANNEL[0].TC_IER = 0;  // No interrupts
  TC0->TC_CHANNEL[0].TC_IDR = ~0; // No interrupts
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN;                    // Enable clock
  TC0->TC_CHANNEL[0].TC_RC = 10000000;                         // Period = 1 second
  TC0->TC_CHANNEL[0].TC_RB = 10000000 - (PPS_OFFSET_NS / 100); // Drive high at top of second
  PIO_Configure(
    PIOB,
    PIO_PERIPH_B,
    PIO_PB27B_TIOB0,
    PIO_DEFAULT
  );

}

static void timers_sync() {
  // Loading this register resets all three channels of TC0 to 0 and starts them
  TC0->TC_BCR = TC_BCR_SYNC;
}

void timers_set_max(uint32_t max) {
  TC0->TC_CHANNEL[0].TC_RC = TC0->TC_CHANNEL[1].TC_RC = max;
}

void TC1_Handler() {
  static int first = 1;
  uint32_t status = TC0->TC_CHANNEL[1].TC_SR;
  if (status & TC_SR_CPCS) { // On RC compare (1Hz)
    second_int();
  }
  if (status & TC_SR_LDRAS) { // On rising edge of PPS
    debug("CAPT: ");
    uint32_t tm = TC0->TC_CHANNEL[1].TC_RA;
    TC0->TC_CHANNEL[1].TC_RB;
    debug(tm); debug("\r\n");
    if (first) {
      timers_sync();
      first = 0;
    } else {
      pps_int = 1;
    }
  }
}

void timer_init() {
  timer0_setup();
  timer1_setup();
  timers_sync();
}
