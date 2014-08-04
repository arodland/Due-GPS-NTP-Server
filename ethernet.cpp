#include "config.h"
#include "debug.h"
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Wire.h>
#include <w5100.h>

#define I2C_ADDRESS 0x50

unsigned char mac[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
volatile char ether_int = 0;
uint32_t packet_ts;

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
    packet_ts = tm;
    ether_int = 1;
  }
}

char packet_buffer[256];

void ether_recv() {
  debug("ETHER INT: ");
  debug(packet_ts);
  debug(" Handled at: ");
  uint32_t tm = TC0->TC_CHANNEL[0].TC_CV;
  debug(tm);
  debug("\r\n");

  int packet_size = Udp.parsePacket();
  do {
    if (packet_size > sizeof(packet_buffer)) {
      debug("Packet too big");
    } else if (packet_size) {
      debug(packet_size);
      debug(" byte packet: ");
      bzero(packet_buffer, sizeof(packet_buffer));
      Udp.read(packet_buffer, sizeof(packet_buffer));
      debug(packet_buffer);
      debug("\r\n");
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      Udp.write(packet_buffer, packet_size);
      Udp.endPacket();
    }
    packet_size = Udp.parsePacket();
  } while (packet_size);
  clear_ether_interrupt();
}
