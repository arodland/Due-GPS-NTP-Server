#include "config.h"
#if GPS_UBLOX

#include <Arduino.h>
#include "gps.h"
#include "timing.h"
#include "debug.h"
#include "health.h"

#define GPS_BUFFER_SIZE 256

enum gps_state_t {
  GPS_SYNC1,     // 0
  GPS_SYNC2,     // 1
  GPS_CLASS,     // 2
  GPS_ID,        // 3
  GPS_LENGTH1,   // 4
  GPS_LENGTH2,   // 5
  GPS_DATA,      // 6
  GPS_CHECKSUM1, // 7
  GPS_CHECKSUM2, // 8
};

static inline int gps_can_read() {
  return GPS.available();
}

static inline int gps_read() {
  return GPS.read();
}

static inline void gps_write(const char *data) {
  GPS.print(data);
}

static inline void gps_writebyte(const char ch) {
  GPS.write(ch);
}

static inline void gps_checksum(const unsigned char ch, unsigned char *ck_a, unsigned char *ck_b) {
  *ck_a += ch;
  *ck_b += *ck_a;
}

static inline void gps_writebyte_check(const unsigned char ch, unsigned char *ck_a, unsigned char *ck_b) {
  GPS.write(ch);
  gps_checksum(ch, ck_a, ck_b);
}

void gps_write_ublox(const unsigned short packetid, const char *packet, int len) {
  unsigned char ck_a = 0, ck_b = 0;

  gps_writebyte(0xb5);
  gps_writebyte(0x62);
  gps_writebyte_check(packetid >> 8, &ck_a, &ck_b);
  gps_writebyte_check(packetid & 0xff, &ck_a, &ck_b);
  gps_writebyte_check(len & 0xff, &ck_a, &ck_b);
  gps_writebyte_check(len >> 16, &ck_a, &ck_b);
  for (int i = 0 ; i < len ; i++) {
    gps_writebyte_check(packet[i], &ck_a, &ck_b);
  }
  gps_writebyte(ck_a);
  gps_writebyte(ck_b);
}

/*
static void gps_set_serial_options() {
  gps_write_tsip(0xBC,
    "\x00" // Port 0
    "\x0A" // Input rate 57600
    "\x0A" // Output rate 57600
    "\x03" // 8 bit
    "\x00" // No parity
    "\x00" // 1 stop bit
    "\x00" // No flow control
    "\x02" // Input TSIP
    "\x02" // Output TSIP
    "\x00" // Reserved
    , 10
  );
  GPS.flush();
  delay(500);
  GPS.begin(57600, SERIAL_8N1);
  GPS.flush();
}
*/

static void gps_set_serial_options() {
  gps_write_ublox(0x600,
      "\x01"     // Port 1
      "\x00"     // Reserved
      "\x00\x00" // txReady disabled
      "",
      4
  );
}

static enum gps_state_t decoder_state = GPS_SYNC1;
static char gps_message_valid;
static unsigned short gps_packetid;
static unsigned int gps_payload_len;
static unsigned int gps_payload_read;
static unsigned char gps_payload[GPS_BUFFER_SIZE];
static unsigned char *gps_payload_ptr;
static unsigned char ck_a, ck_b;

void gps_handle_message();

void gps_poll() {
  while (gps_can_read()) {
    int ch = gps_read();

    switch (decoder_state) {
      case GPS_SYNC1:
//        debug("S1 ");
        if (ch == 0xb5)
          decoder_state = GPS_SYNC2;
        else {
//          debug("Got garbage char "); debug_hex(ch); debug(" from UBX\r\n");
        }
        break;
      case GPS_SYNC2:
//        debug("S2 ");
        if (ch == 0x62) {
          ck_a = ck_b = 0;
          decoder_state = GPS_CLASS;
        } else {
//          debug("Got garbage char "); debug_hex(ch); debug(" from UBX\r\n");
          decoder_state = GPS_SYNC1;
        }
        break;
      case GPS_CLASS:
//        debug("I1 ");
        gps_packetid = ch << 8;
        gps_checksum(ch, &ck_a, &ck_b);
        decoder_state = GPS_ID;
        break;
      case GPS_ID:
//        debug("I2 ");
        gps_packetid |= ch;
        gps_checksum(ch, &ck_a, &ck_b);
        decoder_state = GPS_LENGTH1;
        break;
      case GPS_LENGTH1:
//        debug("L1 ");
        gps_payload_len = ch;
        gps_checksum(ch, &ck_a, &ck_b);
        decoder_state = GPS_LENGTH2;
        break;
      case GPS_LENGTH2:
//        debug("L2 ");
        gps_payload_len |= ch << 8;
        gps_checksum(ch, &ck_a, &ck_b);
        gps_payload_ptr = gps_payload;
        gps_payload_read = 0;
        if (gps_payload_len < GPS_BUFFER_SIZE)
          gps_message_valid = 1;
        else
          gps_message_valid = 0;
        if (gps_payload_len > 0)
          decoder_state = GPS_DATA;
        else
          decoder_state = GPS_CHECKSUM1;
        break;
      case GPS_DATA:
//        debug("DD ");
        if (gps_message_valid)
          *(gps_payload_ptr++) = ch;
        gps_checksum(ch, &ck_a, &ck_b);
        gps_payload_read ++;
        if (gps_payload_read == gps_payload_len)
          decoder_state = GPS_CHECKSUM1;
        break;
      case GPS_CHECKSUM1:
//        debug("C1 ");
        if (ch != ck_a) {
          debug("Invalid CK_A from UBX: "); debug_hex(ch); debug(" != "); debug_hex(ck_a); debug(", message was "); debug_hex(gps_packetid); debug("\r\n");
          gps_message_valid = 0;
        }
        decoder_state = GPS_CHECKSUM2;
        break;
      case GPS_CHECKSUM2:
//        debug("C2 ");
        if (ch != ck_b) {
          debug("Invalid CK_B from UBX: "); debug_hex(ch); debug(" != "); debug_hex(ck_b); debug(", message was "); debug_hex(gps_packetid); debug("\r\n");
          gps_message_valid = 0;
        }
        if (gps_message_valid)
          gps_handle_message();
        decoder_state = GPS_SYNC1;
        break;
    }
//    debug_hex(ch);
//    debug("\r\n");
  }
}

void gps_message_tim_tp() {
  unsigned int tow_msec = *((unsigned int *)(gps_payload));
  unsigned short gps_week = *((unsigned short *)(gps_payload + 12));
  char flags = gps_payload[14];

  time_set_date(gps_week, (tow_msec / 1000), -1);
}

void gps_message_nav_sat() {
}

void gps_message_nav_status() {
  static const char *fix_type_msg[] = {
    "NONE", "DR", "2D", "3D", "GPS+DR", "TIME"
  };

  static const char *flag_msg[] = {
    "FIX_OK", "DGPS", "WN_OK", "TOW_OK"
  };

  unsigned char fix_type = gps_payload[4];
  unsigned char flags = gps_payload[5];

  debug("NAV-STATUS: ");
  debug(fix_type_msg[fix_type]);

  for (int i = 0 ; i < sizeof(flag_msg) / sizeof(*flag_msg) ; i++) {
    if (flags & 1 << i) {
      debug(" ");
      debug(flag_msg[i]);
    }
  }
  debug("\r\n");

  if (flags & 0x0B == 0x0B) {
    health_set_gps_status(GPS_OK);
  } else {
    health_set_gps_status(GPS_UNLOCK);
  }

  health_reset_gps_watchdog();
}

void gps_message_nav_timeutc() {
  unsigned short year = *((unsigned short *)(gps_payload + 12));
  unsigned char month = gps_payload[14];
  unsigned char day = gps_payload[15];
  unsigned char hour = gps_payload[16];
  unsigned char minute = gps_payload[17];
  unsigned char second = gps_payload[18];
  unsigned char flags = gps_payload[19];

  debug("NAV-TIMEUTC: ");
  debug_int(year); debug("-"); debug_int(month); debug("-"); debug_int(day);
  debug(" ");
  debug_int(hour); debug(":"); debug_int(minute); debug(":"); debug_int(second);
  debug("\r\n");
}

void gps_message_nav_clock() {
}

void gps_message_nav_svin() {
}

void gps_handle_message() {
  switch (gps_packetid) {
    case 0x0d01:
      gps_message_tim_tp();
      break;
    case 0x0135:
      gps_message_nav_sat();
      break;
    case 0x0103:
      gps_message_nav_status();
      break;
    case 0x0121:
      gps_message_nav_timeutc();
      break;
    case 0x0122:
      gps_message_nav_clock();
      break;
    case 0x0d04:
      gps_message_nav_svin();
      break;
    default:
      debug("Got "); debug_int(gps_payload_len);
      debug(" byte message, type "); debug_hex(gps_packetid);
      debug("\r\n");
      break;
  }
}

void gps_init() {
  // We don't send any configuration to the unit. It should be configured
  // for fixed position, timing mode (if applicable), 1Hz positive timepulse,
  // and UBX-TIM-TP, UBX-NAV-TIMEUTC, and UBX-NAV-STATUS messages at a minimum.
  GPS.begin(57600, SERIAL_8N1);
}

#endif
