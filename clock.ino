void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  pmc_enable_periph_clk(TC_INTERFACE_ID + 0);
  TC_Configure(TC0, 0,
    TC_CMR_TCCLKS_XC0 |
//    TC_CMR_TCCLKS_TIMER_CLOCK2 |
    TC_CMR_WAVSEL_UP_RC |
    TC_CMR_LDRA_RISING |
    TC_CMR_CPCTRG
  );
  TC0->TC_CHANNEL[0].TC_IER = TC_IER_COVFS | TC_IER_CPCS | TC_IER_LDRAS;
  NVIC_EnableIRQ(TC0_IRQn);
  PIO_Configure(PIOB, PIO_PERIPH_B, PIO_PB25B_TIOA0, PIO_DEFAULT);
  TC_SetRC(TC0, 0, 10000000);
  TC_Start(TC0, 0);
}

void TC0_Handler() {
  static int ct = 0;
  uint32_t status = TC0->TC_CHANNEL[0].TC_SR;
  if (status & TC_SR_CPCS) {
    Serial.println(ct++);
  }
  if (status & TC_SR_COVFS) {
    Serial.println("OVERFLOW!");
  }
  if (status & TC_SR_LDRAS) {
    Serial.print("CAPT: ");
    uint32_t tm = TC0->TC_CHANNEL[0].TC_RA;
    TC0->TC_CHANNEL[0].TC_RB;
    Serial.println(tm);
  }
}

void loop() {
}
