#include "config.h"

#if MONITOR_ENABLED

#include "timing.h"
#include "ethernet.h"

void monitor_send(const char *metric, int value) {
  char buf[512] = MONITOR_PREFIX;
  const char ip[4] = {MONITOR_IP_ADDRESS};
  const char mac[6] = {MONITOR_MAC_ADDRESS};
  strcat(buf, metric);
  strcat(buf, " ");
  itoa(value, buf + strlen(buf), 10);
  strcat(buf, " ");
  itoa(time_get_unix(), buf + strlen(buf), 10);
  ethernet_send_udp_packet(ip, mac, MONITOR_PORT, MONITOR_PORT, buf, strlen(buf));
}

#else

void monitor_send(const char *metric, int value) {
  /* empty */
}

#endif
