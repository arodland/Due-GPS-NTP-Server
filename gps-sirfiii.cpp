#include "config.h"
#if GPS_SIRFIII

#include <Arduino.h>
#include "gps.h"
#include "timing.h"
#include "debug.h"

#define GPS_BUFFER_SIZE 256

enum gps_state_t {
  GPS_HEADER1,    // 0
  GPS_HEADER2,    // 1
  GPS_LENGTH1,    // 2
  GPS_LENGTH2,    // 3
  GPS_PAYLOAD,    // 4
  GPS_CHECKSUM1,  // 5
  GPS_CHECKSUM2,  // 6
  GPS_TRAILER1,   // 7
  GPS_TRAILER2,   // 8
};

static void gps_set_sirf();
static void gps_enable_dgps();

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

static inline void gps_set_baud(long baud) {
  GPS.flush();
  delay(500);
  GPS.begin(baud);
  GPS.flush();
}

void gps_init() {
  GPS.begin(4800);;
  gps_set_sirf();
  gps_enable_dgps();
}

inline char to_hex(unsigned char val) {
  if (val > 10) {
    return 'A' + (val - 10);
  } else {
    return '0' + val;
  }
}

void gps_write_nmea(const char *sentence) {
  unsigned char cksum = 0;
  for (const char *i = sentence; *i; i++) {
    cksum ^= *i;
  }
  gps_writebyte('$');
  gps_write(sentence);
  gps_writebyte('*');
  gps_writebyte(to_hex(cksum >> 4));
  gps_writebyte(to_hex(cksum & 0x0F));
  gps_writebyte('\x0d');
  gps_writebyte('\x0a');
}

void gps_write_sirf(const char *sentence, int len) {
  unsigned short cksum = 0;
  gps_write("\xa0\xa2");
  gps_writebyte(len >> 8);
  gps_writebyte(len & 0xff);
  for (int i = 0 ; i < len ; i++) {
    gps_writebyte(sentence[i]);
    cksum += (unsigned char) sentence[i];
  }
  gps_writebyte((cksum & 0x7fff) >> 8);
  gps_writebyte(cksum & 0xff);
  gps_write("\xb0\xb3");
}

void gps_enable_dgps() {
  gps_write_sirf("\x84\x00", 2);
  /* Cmd: 128, 22 bytes unused, 20 channels, enable navlib */
  // gps_write_sirf("\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x14\x10", 25);
  /* Cmd: 128, 22 bytes unused, 20 channels, disable navlib */
//  gps_write_sirf("\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x14\x00", 25);

  /* Cmd: 133, DGPS source: 1, 5 bytes unused */
  gps_write_sirf("\x85\x01\x00\x00\x00\x00\x00", 7);
  gps_write_sirf("\x85\x01\x00\x00\x00\x00\x00", 7);
  /* Cmd: 138, DGPS selection: auto, DGPS timeout: auto */
  gps_write_sirf("\x8a\x00\x00", 3);
  /* Cmd: 170, PRN: auto, mode: testing, timeout: auto, 2 bytes unused */
  //gps_write_sirf("\xaa\x00\x00\x00\x00\x00", 6);
  /* Cmd: 170, PRN: 138, mode: testing, timeout: auto, 2 bytes unused */
  gps_write_sirf("\xaa\x8a\x00\x00\x00\x00", 6);
  /* Cmd: 166, Mode: Enable one, Message ID: 8, Rate: 1Hz, 4 bytes unused */
  gps_write_sirf("\xa0\x00\x08\x01\x00\x00\x00\x00", 8);
  /* Cmd: 166, Mode: Enable one, Message ID: 7, Rate: 10sec, 4 bytes unused */
  gps_write_sirf("\xa0\x00\x07\x0A\x00\x00\x00\x00", 8);
  /* Cmd: 166, Mode: Enable one, Message ID: 4, Rate: 1Hz, 4 bytes unused */
  gps_write_sirf("\xa0\x00\x04\x01\x00\x00\x00\x00", 8);

}

void gps_set_nmea_reporting() {
  gps_write_nmea("PSRF103,00,00,01,01"); // GPGGA 1sec
  gps_write_nmea("PSRF103,01,00,01,01"); // GPGLL off
  gps_write_nmea("PSRF103,02,00,01,01"); // GPGSA 1sec
  gps_write_nmea("PSRF103,03,00,10,01"); // GPGSV 10sec
  gps_write_nmea("PSRF103,04,00,01,01"); // GPRMC 1sec
  gps_write_nmea("PSRF103,05,00,00,01"); // GPVTG off
  gps_write_nmea("PSRF103,07,00,01,01"); // GPZDA 1sec
  gps_write_nmea("PSRF151,01"); // Enable WAAS */
}

void gps_set_sirf() {
//  gps_write_nmea("PSRF101,0,0,0,000,0,0,12,8");
  delay(1000);
  gps_write_nmea("PSRF100,0,38400,8,1,0");
  gps_set_baud(38400);
}

static enum gps_state_t decoder_state;
static char gps_ignore_payload;
static char gps_message_valid;
static unsigned int gps_payload_len;
static unsigned int gps_payload_remain;
static unsigned char gps_payload[GPS_BUFFER_SIZE];
static unsigned int gps_payload_checksum;
static unsigned int msg_checksum;
static unsigned char *gps_payload_ptr;

int gps_utc_offset(unsigned int hour, unsigned int minute, unsigned int second, unsigned int tow_second) {
  unsigned int utc_tod = hour * 3600L + minute * 60L + second;
  unsigned int gps_tod = tow_second % 86400L;
  int utc_offset = utc_tod - gps_tod;

  if (utc_offset > 43200L)
    utc_offset -= 86400L;
  if (utc_offset < -43200L)
    utc_offset += 86400L;

  return utc_offset;
}

void gps_handle_message();

void gps_poll() {
  while (gps_can_read()) {
    int ch = gps_read();
    switch (decoder_state) {
      case GPS_HEADER1:
        if (ch == 0xa0)
          decoder_state = GPS_HEADER2;
        else {
          debug("Got garbage char "); debug_int(ch); debug(" from GPS\r\n");
        }
        break;
      case GPS_HEADER2:
        if (ch == 0xa2)
          decoder_state = GPS_LENGTH1;
        else {
          debug("Got garbage char "); debug_int(ch); debug(" from GPS (expecting 2nd header byte)\r\n");
          decoder_state = GPS_HEADER1;
        }
        break;
      case GPS_LENGTH1:
        gps_payload_len = (ch & 0x7f) << 8;
        decoder_state = GPS_LENGTH2;
        break;
      case GPS_LENGTH2:
        gps_payload_len += ch;
        gps_payload_remain = gps_payload_len;
        if (gps_payload_len > GPS_BUFFER_SIZE - 1) {
          debug_int(gps_payload_len); debug(" byte payload too big\r\n");
          gps_ignore_payload = 1;
        } else {
          gps_message_valid = 1;
          gps_ignore_payload = 0;
          gps_payload_checksum = 0;
          gps_payload_ptr = gps_payload;
        }
        decoder_state = GPS_PAYLOAD;
        break;
      case GPS_PAYLOAD:
        if (!gps_ignore_payload) {
          *(gps_payload_ptr++) = ch;
          gps_payload_checksum += ch;
        }
        if (--gps_payload_remain == 0)
          decoder_state = GPS_CHECKSUM1;
        break;
      case GPS_CHECKSUM1:
        msg_checksum = ch << 8;
        decoder_state = GPS_CHECKSUM2;
        break;
      case GPS_CHECKSUM2:
        msg_checksum += ch;
        if (!gps_ignore_payload) {
          gps_payload_checksum &= 0x7FFF;
          if (gps_payload_checksum != msg_checksum) {
            debug("Received checksum "); debug_int(msg_checksum);
            debug(" != calculated "); debug_int(gps_payload_checksum);
            debug(", discarding message\r\n");
            gps_message_valid = 0;
          }
        }
        decoder_state = GPS_TRAILER1;
        break;
      case GPS_TRAILER1:
        if (ch == 0xb0)
          decoder_state = GPS_TRAILER2;
        else {
          debug("GPS got garbage "); debug_int(ch); debug(" (expecting 1st trailer)\r\n");
          decoder_state = GPS_HEADER1;
        }
        break;
      case GPS_TRAILER2:
        if (ch == 0xb3) {
          if (gps_message_valid)
            gps_handle_message();
        } else {
          debug("GPS Got garbage "); debug_int(ch); debug(" (expecting 2nd trailer\r\n");
        }
        decoder_state = GPS_HEADER1;
        break;
      default:
        debug("GPS decoder in unknown state?\r\n");
        decoder_state = GPS_HEADER1;
    }
  }
}

void gps_navdata_message() {
#if 0
  unsigned int mode1 = gps_payload[19];
  unsigned int mode2 = gps_payload[21];
  unsigned int svs = gps_payload[28];
  unsigned int pmode = mode1 & 0x7;
  debug("DGPS: ");
  if (mode1 & 1<<7)
    debug("OK");
  else
    debug("NO");

  debug(" DOP: ");
  if (mode1 & 1<<6)
    debug("NO");
  else
    debug("OK");

  debug(" VAL: ");
  if (mode2 & 1<<1)
    debug("OK");
  else
    debug("NO");
  debug(" PMODE: ");
  debug_int(pmode);
  debug("\n");
#endif
}

void gps_tracking_data_message() {
#if 0
  unsigned char chans = gps_payload[7];
  unsigned char ch;
  unsigned char first = 1;

  debug("SVs in view: ");

  for (ch = 0 ; ch < chans ; ch++) {
    unsigned char svid = gps_payload[8 + ch * 15];
    if (svid != 0) {
      unsigned char raw_az = gps_payload[9 + ch * 15];
      unsigned char raw_el = gps_payload[10 + ch * 15];
      unsigned int state = (unsigned int) gps_payload[11 + ch * 15] << 8
                        | (unsigned int) gps_payload[12 + ch * 15];
      float az = raw_az * 3.0 / 2.0;
      float el = raw_el / 2.0;
      float snr_avg = 0;
      unsigned char meas;
      for (meas = 0 ; meas < 2 ; meas++) {
        snr_avg += gps_payload[15 + ch * 15 + meas];
      }
      snr_avg /= 2.0;

      if (first)
        first = 0;
      else
        debug(" ");
      debug_int((unsigned int)svid);
      debug(" ["); debug_float(az);
      debug(" "); debug_float(el);
      debug(" "); debug_float(snr_avg);
      debug("]");
    }
  }
  debug("\n");
#endif
}

void gps_clockstatus_message() {
#if 0
  unsigned int clock_bias = (unsigned int)gps_payload[12] << 24
                 | (unsigned int)gps_payload[13] << 16
                 | (unsigned int)gps_payload[14] << 8
                 | (unsigned int)gps_payload[15];
  unsigned int gps_ms = (unsigned int)gps_payload[16] << 24
                 | (unsigned int)gps_payload[17] << 16
                 | (unsigned int)gps_payload[18] << 8
                 | (unsigned int)gps_payload[19];
  debug("Clock bias: "); debug_long(clock_bias); debug("\n");
  debug("GPS Time ms: "); debug_long(gps_ms); debug("\n");
#endif
}

void gps_satvisible_message() {
#if 0
  unsigned short num_visible = gps_payload[1];
  debug("Visible SVs:");
  for (int i = 0 ; i < num_visible; i++) {
    debug(" ");
    debug_int((int)(gps_payload[2 + 5 * i]));
  }
  debug("\n");
#endif
}

void gps_dgps_message() {
#if 0
  unsigned int dgps_source = gps_payload[1];

  debug("DGPS source: "); debug_int(dgps_source); debug(", PRN:");
  for (int i = 16; i < 52 ; i += 3) {
    debug(" ");
    debug_int((int)(gps_payload[i]));
  }
  debug("\n");
#endif
}

void gps_geodetic_message() {
  unsigned int year = gps_payload[11] << 8 | gps_payload[12];
  unsigned int month = gps_payload[13];
  unsigned int day = gps_payload[14];
  unsigned int hour = gps_payload[15];
  unsigned int minute = gps_payload[16];
  unsigned int rawsecond = (gps_payload[17] << 8 | gps_payload[18]);
  unsigned int second = rawsecond / 1000;
  unsigned int millis = rawsecond % 1000;

  unsigned int gps_week = gps_payload[5] << 8 | gps_payload[6];
  unsigned int gps_tow = (unsigned int)gps_payload[7] << 24
                 | (unsigned int)gps_payload[8] << 16
                 | (unsigned int)gps_payload[9] << 8
                 | (unsigned int)gps_payload[10];
  unsigned int gps_tow_sec = gps_tow / 1000L;

  unsigned int numsvs = gps_payload[88];

  debug_int(year); debug("-"); debug_int(month); debug("-"); debug_int(day);
  debug(" ");
  debug_int(hour); debug(":"); debug_int(minute); debug(":");
  debug_int(second); debug("."); debug_int(millis);
  debug(", ");
  debug_int(numsvs);
  debug(" SVs\r\n");

  int utc_offset = gps_utc_offset(hour, minute, second, gps_tow_sec);

  time_set_date(gps_week, gps_tow_sec, utc_offset);
}

void gps_ack_message() {
  debug("Got ACK for message ");
  debug_int((int)gps_payload[1]);
  debug("\r\n");
}

void gps_nak_message() {
  debug("Got NAK for message ");
  debug_int((int)gps_payload[1]);
  debug("\r\n");
}
void gps_handle_message() {
  unsigned char message_type = gps_payload[0];
  switch(message_type) {
    case 2:
      gps_navdata_message();
      break;
    case 4:
      gps_tracking_data_message();
      break;
    case 7:
      gps_clockstatus_message();
      break;
    case 11:
      gps_ack_message();
      break;
    case 12:
      gps_nak_message();
      break;
    case 13:
      gps_satvisible_message();
      break;
    case 27:
      gps_dgps_message();
      break;
    case 41:
      gps_geodetic_message();
      break;
    case 9:
      break;
    default:
      debug("Got "); debug_int(gps_payload_len);
      debug(" byte message, type "); debug_int((int)message_type);
      debug("\r\n");
      break;
  }
}

#endif
