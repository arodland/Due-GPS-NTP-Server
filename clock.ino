#include "timer.h"
#include "gps.h"
#include "ethernet.h"
#include "console.h"

void setup() {
  console_init();
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
