#include "config.h"

#if MONITOR_ENABLED

#include "timing.h"
#include "ethernet.h"

static char monitor_packet[1024];

void monitor_flush() {
  const char ip[4] = {MONITOR_IP_ADDRESS};
  const char mac[6] = {MONITOR_MAC_ADDRESS};

  size_t packet_size = strlen(monitor_packet);
  if (packet_size == 0)
    return;

  ethernet_send_udp_packet(ip, mac, MONITOR_PORT, MONITOR_PORT, monitor_packet, packet_size);
  monitor_packet[0] = '\0';
}

static void monitor_append(const char *buf) {
  if (strlen(buf) + strlen(monitor_packet) >= sizeof(monitor_packet)) {
    monitor_flush();
  }
  strcat(monitor_packet, buf);
}

void monitor_send(const char *metric, int value) {
  char buf[512] = MONITOR_PREFIX;
  strcat(buf, metric);
  strcat(buf, " ");
  itoa(value, buf + strlen(buf), 10);
  strcat(buf, " ");
  itoa(time_get_unix(), buf + strlen(buf), 10);
  strcat(buf, "\n");
  monitor_append(buf);
}

void monitor_sendf(const char *metric, float value) {
  char buf[512] = MONITOR_PREFIX;
  strcat(buf, metric);
  strcat(buf, " ");
  sprintf(buf + strlen(buf), "%f", value);
  strcat(buf, " ");
  itoa(time_get_unix(), buf + strlen(buf), 10);
  strcat(buf, "\n");
  monitor_append(buf);
}

#else

void monitor_send(const char *metric, int value) {
  /* empty */
}

void monitor_sendf(const char *metric, float value) {
  /* empty */
}
#endif
