#include <Arduino.h>

#define Console Serial
#define GPS Serial1
#define Rb Serial2

#define GPS_SIRFIII 0
#define GPS_TSIP 0
#define GPS_UBLOX 1
#define GPS_UBLOX_TIMESTAMP 1

#define DEBUG 1

#define DHCP 0
#define IPADDRESS 192,168,1,202

#define HZ 30000000L
#define NSPT (1000000000L/HZ)
#define NTP_ROUND_MAGIC 0x9545

#define TIMER_CLOCK (&(TC0->TC_CHANNEL[1].TC_CV))
#define TIMER_CAPT_PPS (&(TC0->TC_CHANNEL[1].TC_RA))
//#define TIMER_CAPT_ETHER (&(TC0->TC_CHANNEL[0].TC_RA))

#define PPS_OFFSET_NS 1000000
#define PPS_OFFSET_NTP 4294967

#define PPS_FUDGE_NS 0

#define PPSOUT_OFFSET_NS 0

#define NTP_FUDGE_US 110
#define NTP_FUDGE_NTP 472446

#define NTP_FUDGE_RX_US -10
#define NTP_FUDGE_TX_US 20

#define FLL_START_VALUE 100
#define FLL_MIN_FACTOR 1800
#define FLL_MAX_FACTOR 10800
#define FLL_SMOOTH 1
#define FLL_MAX 2000

#define PLL_STARTUP_THRESHOLD 80000
#define PLL_MIN_FACTOR 600
#define PLL_MAX_FACTOR 10800

#define PPS_FILTER_MIN 8
#define PPS_FILTER_MAX 24
#define PPS_FILTER_DIV 300

#define PLL_HEALTHY_THRESHOLD_NS 1000 /* PLL is synced if within this range */
#define HOLDOVER_LIMIT_SEC 86400 /* Can holdover for this many seconds after having a valid frequency */

#define MONITOR_ENABLED 1
#define MONITOR_IP_ADDRESS 192,168,1,3
#define MONITOR_MAC_ADDRESS 0x78,0x8a,0x20,0xba,0x29,0xc9
#define MONITOR_PORT 2003
#define MONITOR_PREFIX "duet."

#define CONSOLE_OUTBUF_SIZE 512
#define CONSOLE_CMDLINE_SIZE 512

