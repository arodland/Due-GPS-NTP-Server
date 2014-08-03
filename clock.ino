#include "config.h"
#include "timer.h"
#include "gps.h"
#include "ethernet.h"
#include "console.h"
#include "rb.h"

void setup() {
  console_init();
  timer_init();
  gps_init();
  rb_init();
  ether_init();
}

void loop() {
  gps_poll();
  if (ether_int) {
    ether_recv();
  }
  if (Rb.available()) {
    Console.write(Rb.read());
  }
  if (Console.available()) {
    Rb.write(Console.read());
  }
}
