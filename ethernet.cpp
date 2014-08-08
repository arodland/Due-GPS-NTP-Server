#include "config.h"
#include "debug.h"
#include "timing.h"
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Wire.h>
#include <w5100.h>

#define I2C_ADDRESS 0x50

unsigned char mac[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
volatile char ether_int = 0;
uint32_t recv_ts_upper, recv_ts_lower;

EthernetUDP Udp;

void get_mac_address() {
  int i;
  byte reg;
  debug("Getting MAC address: ");
  Wire.begin();
  for (i=0, reg=0xFA; i < 6; i++, reg++) {
    Wire.beginTransmission(I2C_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(I2C_ADDRESS, 1);
    while (!Wire.available()) {
    }
    mac[i] = Wire.read();
    debug_hex(mac[i]);
    if (i < 5)
      debug(":");
  }
  debug("\r\n");
}


void ether_init() {
  get_mac_address();
  delay(250);
#if DHCP
  int ret = Ethernet.begin(mac);
  if (ret) {
    IPAddress ip = Ethernet.localIP();
    debug("IP address: ");
    for (int i = 0 ; i < 4 ; i++) {
      debug(ip[i]);
      if (i < 3)
        debug(".");
      else
        debug("\r\n");
    }
  } else {
    debug("DHCP failed, Ethernet unavailable");
  }
#else
  IPAddress ip(IPADDRESS);
  Ethernet.begin(mac, ip);
#endif
  W5100.writeIMR(0x0F); // Enable interrupts
  Udp.begin(123);
}

void clear_ether_interrupt() {
  __disable_irq();
  W5100.writeIR(0xF0);
  W5100.writeSnIR(0, 0xFF);
  W5100.writeSnIR(1, 0xFF);
  W5100.writeSnIR(2, 0xFF);
  W5100.writeSnIR(3, 0xFF);
  __enable_irq();
  ether_int = 0;
}

void ether_interrupt(uint32_t tm) {
  if (!ether_int) {
    time_get_ntp(tm, &recv_ts_upper, &recv_ts_lower, 0);
    ether_int = 1;
  }
}

static const char ntp_packet_template[48] = {
  4 /* Mode: server reply */ | 3 << 3 /* Version: NTPv3 */,
  1 /* Stratum */, 9 /* Poll: 512sec */, -23 /* Precision: 0.1 usec */,
  0, 0, 0, 0 /* Root delay */,
  0, 0, 0, 10 /* Root Dispersion */,
  'G', 'P', 'S', 0 /* Reference ID */,
  0, 0, 0, 0, 0, 0, 0, 0 /* Reference Timestamp */,
  0, 0, 0, 0, 0, 0, 0, 0 /* Origin Timestamp */,
  0, 0, 0, 0, 0, 0, 0, 0 /* Receive Timestamp */,
  0, 0, 0, 0, 0, 0, 0, 0 /* Transmit Timestamp */
};

void do_ntp_request(unsigned char *buf, unsigned int len,
    IPAddress ip, uint16_t port) {
  unsigned char version = (buf[0] >> 3) & 7;
  unsigned char mode = buf[0] & 7;

  if (len < 48) {
    debug("Not NTP\r\n");
    return;
  }

  if (version != 3 && version != 4) {
    debug("NTP unknown version\r\n");
    return;
  }

  if (mode == 3) { /* Client request */
    unsigned char reply[48];
    uint32_t tx_ts_upper, tx_ts_lower;
    memcpy(reply, ntp_packet_template, 48);
    /* XXX set Leap Indicator */
    /* Copy client transmit timestamp into origin timestamp */
    memcpy(reply + 24, buf + 40, 8);
    /* Copy receive timestamp into packet */
    reply[32] = (recv_ts_upper >> 24) & 0xff;
    reply[33] = (recv_ts_upper >> 16) & 0xff;
    reply[34] = (recv_ts_upper >> 8) & 0xff;
    reply[35] = (recv_ts_upper) & 0xff;
    reply[36] = (recv_ts_lower >> 24) & 0xff;
    reply[37] = (recv_ts_lower >> 16) & 0xff;
    reply[38] = (recv_ts_lower >> 8) & 0xff;
    reply[39] = (recv_ts_lower) & 0xff;
    /* Copy top half of receive timestamp into reference timestamp --
     * we update clock every second :)
     */
    memcpy(reply + 16, reply + 32, 4);

    time_get_ntp(*TIMER_CLOCK, &tx_ts_upper, &tx_ts_lower, 0);
    /* Copy tx timestamp into packet */
    reply[40] = (tx_ts_upper >> 24) & 0xff;
    reply[41] = (tx_ts_upper >> 16) & 0xff;
    reply[42] = (tx_ts_upper >> 8) & 0xff;
    reply[43] = (tx_ts_upper) & 0xff;
    reply[44] = (tx_ts_lower >> 24) & 0xff;
    reply[45] = (tx_ts_lower >> 16) & 0xff;
    reply[46] = (tx_ts_lower >> 8) & 0xff;
    reply[47] = (tx_ts_lower) & 0xff;

    Udp.beginPacket(ip, port);
    Udp.write(reply, 48);
    Udp.endPacket();
  } else {
    debug("NTP unknown packet type\r\n");
  }
}


unsigned char packet_buffer[256];

void ether_recv() {
  if (ether_int)
    debug("ETHER INT\r\n");

  int packet_size = Udp.parsePacket();
  do {
    if (packet_size > sizeof(packet_buffer)) {
      debug("Packet too big");
    } else if (packet_size) {
      bzero(packet_buffer, sizeof(packet_buffer));
      Udp.read(packet_buffer, sizeof(packet_buffer));
      do_ntp_request(packet_buffer, packet_size, Udp.remoteIP(), Udp.remotePort());
    }
    packet_size = Udp.parsePacket();
  } while (packet_size);
  clear_ether_interrupt();
}

