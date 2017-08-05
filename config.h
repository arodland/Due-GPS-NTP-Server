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

#define PPS_OFFSET_NS 1275616
#define PPS_OFFSET_NTP 5478729

#define PPS_FUDGE_NS 50

#define PPSOUT_OFFSET_NS 128

#define NTP_FUDGE_US 110
#define NTP_FUDGE_NTP 472446

#define NTP_FUDGE_RX_US -10
#define NTP_FUDGE_TX_US 20

#define FLL_START_VALUE -250
#define FLL_MIN_FACTOR 1800
#define FLL_MAX_FACTOR 3600
#define FLL_SMOOTH 1
#define FLL_MAX 2000

#define PLL_STARTUP_THRESHOLD 80000
#define PLL_MIN_FACTOR 600
#define PLL_MAX_FACTOR 5400

#define PPS_FILTER_FACTOR 32

#define PLL_HEALTHY_THRESHOLD_NS 1000 /* PLL is synced if within this range */
#define HOLDOVER_LIMIT_SEC 86400 /* Can holdover for this many seconds after having a valid frequency */

#define MONITOR_ENABLED 1
#define MONITOR_IP_ADDRESS 192,168,1,3
#define MONITOR_MAC_ADDRESS 0xb4,0x75,0x0e,0x5c,0x5d,0xcc
#define MONITOR_PORT 1234
#define MONITOR_PREFIX "duet."

#define CONSOLE_OUTBUF_SIZE 512
#define CONSOLE_CMDLINE_SIZE 512

