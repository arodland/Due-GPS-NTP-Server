#include <Arduino.h>

#define Console Serial
#define GPS Serial1
#define Rb Serial2

#define GPS_SIRFIII 0
#define GPS_TSIP 1

#define DEBUG 1

#define DHCP 0
#define IPADDRESS 192,168,1,202

#define PLL_FUDGE_NS 0

#define TIMER_CLOCK (&(TC0->TC_CHANNEL[1].TC_CV))
#define TIMER_CAPT_PPS (&(TC0->TC_CHANNEL[1].TC_RA))
//#define TIMER_CAPT_ETHER (&(TC0->TC_CHANNEL[0].TC_RA))

#define PPS_OFFSET_NS 1345400
#define PPS_OFFSET_NTP 5778449
#define PPS_FUDGE_NS 0

#define NTP_FUDGE_RX_US -50
#define NTP_FUDGE_TX_US 1100

#define FLL_MIN_LEN 120
#define FLL_MAX_LEN 3600

#define PLL_STARTUP_THRESHOLD 100000
#define PLL_MIN_FACTOR 20
#define PLL_MAX_FACTOR 3600
