#include "config.h"
#include "timer.h"
#include "timing.h"
#include "gps.h"
#include "ethernet.h"
#include "console.h"
#include "rb.h"
#include "health.h"
#include "monitor.h"

void setup() {
  console_init();
  timer_init();
  gps_init();
  rb_init();
  ether_init();
}

static char pll_was_running = 0;

void loop() {
  if (pps_int) {
    pps_int = 0;
    char run_pll = health_should_run_pll();
    if (run_pll) {
      if (!pll_was_running) {
        pll_reset_state();
      }
      pll_run();
    }
    pll_was_running = run_pll;
    ethernet_send_ntp_stats();
    monitor_flush();
  }
  console_write_buffered();
  gps_poll();
  if (ether_int) {
    ether_recv();
  }
  console_handle_input();
}
