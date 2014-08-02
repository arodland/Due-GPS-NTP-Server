#include "timer.h"
#include "gps.h"
#include "ethernet.h"

void setup() {
  SerialUSB.begin(115200);
  timer_init();
  gps_init();
  ether_init();
}

void loop() {
  gps_poll();
  if (ether_int) {
    ether_recv();
  }
}
