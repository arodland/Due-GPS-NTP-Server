#include "config.h"
#include "debug.h"
#include "ethernet.h"
#include "timing.h"

volatile char pps_int = 0;
static char pps_output_enabled = 0;

static void timer1_setup() {
  pmc_enable_periph_clk(ID_TC1); // TC0 Ch1
  TC_Configure(TC0, 1,
    TC_CMR_TCCLKS_XC0 |          // XC0 = TCLK0 = PB26 = pin 22
    TC_CMR_WAVSEL_UP_RC |        // Count up to RC then reset to 0
    TC_CMR_LDRA_RISING           // Load RA and generate interrupt on rising edge of TIOA1 (PA2 = pin A7)
  );
  TC0->TC_CHANNEL[1].TC_IER = TC_IER_LDRAS | TC_IER_CPCS;    // Generate interrupt on RA load and 1Hz
  TC0->TC_CHANNEL[1].TC_IDR = ~(TC_IER_LDRAS | TC_IER_CPCS);
  TC0->TC_CHANNEL[1].TC_CCR = TC_CCR_CLKEN;    // Enable clock
  TC0->TC_CHANNEL[1].TC_RC = 31250000;         // Period = 1 second
  NVIC_EnableIRQ(TC1_IRQn);                    // Enable IRQ (TC1_Handler)
}

static void timer0_setup() {
  pmc_enable_periph_clk(ID_TC0);
  TC_Configure(TC0, 0,
    TC_CMR_TCCLKS_XC0 |          // XC0 = TCLK0 = PB26 = pin 22
    TC_CMR_WAVSEL_UP_RC |        // Count up to RC then reset to 0
    TC_CMR_WAVE |                // Generate output waveform
    TC_CMR_EEVT_XC1              // Let TIOB be an output
  );
  TC0->TC_CHANNEL[0].TC_IER = 0;  // No interrupts
  TC0->TC_CHANNEL[0].TC_IDR = ~0; // No interrupts
  TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN;                    // Enable clock
  TC0->TC_CHANNEL[0].TC_RC = 31250000;                         // Period = 1 second
  TC0->TC_CHANNEL[0].TC_RA = 31250000 - ((PPS_OFFSET_NS - PPSOUT_OFFSET_NS) / 32) - 1; // Drive high at top of second
  TC0->TC_CHANNEL[0].TC_RB = 31250000 - ((PPS_OFFSET_NS - PPSOUT_OFFSET_NS) / 32) - 1; // Drive high at top of second
  PIO_Configure(
    PIOB,
    PIO_PERIPH_B,
    PIO_PB27B_TIOB0,
    PIO_DEFAULT
  );
  PIO_Configure(
    PIOB,
    PIO_PERIPH_B,
    PIO_PB25B_TIOA0,
    PIO_DEFAULT
  );
}

static void timers_start() {
  // Loading this register resets all three channels of TC0 to 0 and starts them
  TC0->TC_BCR = TC_BCR_SYNC;
}

static void timers_sync() {
  do { } while (! (TC0->TC_CHANNEL[1].TC_SR & TC_SR_CPCS));
  int32_t tgt = TC0->TC_CHANNEL[1].TC_RA + (PPS_OFFSET_NS + PPS_FUDGE_NS) / 32 - 7;
  if (tgt < 0)
    tgt += 31250000;
  if (tgt >= 31250000)
    tgt -= 31250000;

  int32_t diff;
  do {
    diff = TC0->TC_CHANNEL[1].TC_CV - tgt;
  } while (diff > 1 || diff < -1);
  TC0->TC_BCR = TC_BCR_SYNC;
}

void timers_set_max(uint32_t max) {
  TC0->TC_CHANNEL[0].TC_RC = TC0->TC_CHANNEL[1].TC_RC = max;
}

static int jam_sync = 1;

void timers_jam_sync() {
  jam_sync = 1;
}

void TC1_Handler() {
  uint32_t status = TC0->TC_CHANNEL[1].TC_SR;
  if (status & TC_SR_CPCS) { // On RC compare (1Hz)
    second_int();
  }
  if (status & TC_SR_LDRAS) { // On rising edge of PPS
    debug("CAPT: ");
    uint32_t tm = TC0->TC_CHANNEL[1].TC_RA;
    TC0->TC_CHANNEL[1].TC_RB;
    debug(tm); debug("\r\n");
    if (jam_sync) {
      timers_sync();
      jam_sync = 0;
    } else {
      pps_int = 1;
    }
  }
}

void timer_init() {
  pinMode(13, OUTPUT);
  pinMode(2, OUTPUT);
  timer0_setup();
  timer1_setup();
  timers_start();
}

void pps_output_enable() {
  if (!pps_output_enabled) {
    TC0->TC_CHANNEL[0].TC_CMR |= (TC_CMR_ACPA_SET | TC_CMR_ACPC_CLEAR | TC_CMR_BCPB_SET | TC_CMR_BCPC_CLEAR);
    pps_output_enabled = 1;
  }
}

void pps_output_disable() {
  if (pps_output_enabled) {
    TC0->TC_CHANNEL[0].TC_CMR &= ~(TC_CMR_ACPA_SET | TC_CMR_ACPC_CLEAR | TC_CMR_BCPB_SET | TC_CMR_BCPC_CLEAR);
    pps_output_enabled = 0;
  }
}
