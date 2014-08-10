#include <Arduino.h>

#define Console Serial
#define GPS Serial1
#define Rb Serial2

#define DEBUG 1

#define DHCP 0
#define IPADDRESS 192,168,1,202

#define PLL_FUDGE_NS 0

#define TIMER_CLOCK (&(TC0->TC_CHANNEL[1].TC_CV))
#define TIMER_CAPT_PPS (&(TC0->TC_CHANNEL[1].TC_RA))
//#define TIMER_CAPT_ETHER (&(TC0->TC_CHANNEL[0].TC_RA))

