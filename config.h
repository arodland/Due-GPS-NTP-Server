#include <Arduino.h>

#define Console Serial
#define GPS Serial1
#define Rb Serial2

#define GPS_SIRFIII 0
#define GPS_TSIP 1

#define DEBUG 1

#define DHCP 0
#define IPADDRESS 192,168,1,202

#define TIMER_CLOCK (&(TC0->TC_CHANNEL[1].TC_CV))
#define TIMER_CAPT_PPS (&(TC0->TC_CHANNEL[1].TC_RA))
//#define TIMER_CAPT_ETHER (&(TC0->TC_CHANNEL[0].TC_RA))

#define PPS_OFFSET_NS 1345400
#define PPS_OFFSET_NTP 5778449
#define PPS_FUDGE_NS -50
#define PPSOUT_OFFSET_NS 10000

#define NTP_FUDGE_RX_US -200
#define NTP_FUDGE_TX_US 850

#define FLL_MIN_LEN 300
#define FLL_MAX_LEN 1800

#define PLL_STARTUP_THRESHOLD 80000
#define PLL_MIN_FACTOR 20
#define PLL_MAX_FACTOR 3600

#define PLL_HEALTHY_THRESHOLD_NS 1000 /* PLL is synced if within this range */
#define HOLDOVER_LIMIT_SEC 86400 /* Can holdover for this many seconds after having a valid frequency */
