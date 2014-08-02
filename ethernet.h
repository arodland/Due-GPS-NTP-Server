#ifndef __ETHERNET_H
#define __ETHERNET_H

extern void ether_init();
extern void ether_interrupt(uint32_t tm);
extern void ether_recv();

extern volatile char ether_int;

#endif
