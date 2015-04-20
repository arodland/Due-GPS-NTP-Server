#ifndef __ETHERNET_H
#define __ETHERNET_H

extern void ether_init();
extern void ether_interrupt(uint32_t tm);
extern void ether_recv();

extern void ethernet_send_udp_packet(const char[], const char[], uint16_t, uint16_t, const char *, unsigned int);

extern volatile char ether_int;

#endif
