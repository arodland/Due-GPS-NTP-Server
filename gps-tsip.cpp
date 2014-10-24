#include "config.h"
#if GPS_TSIP

#include <Arduino.h>
#include "gps.h"
#include "timing.h"
#include "debug.h"
#include "health.h"

#define GPS_BUFFER_SIZE 256

enum gps_state_t {
  GPS_LEADER,    // 0
  GPS_PACKETID,  // 1
  GPS_PACKETID2, // 2
  GPS_DATA,      // 2
  GPS_DLE,       // 3
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

void gps_write_tsip(const unsigned short packetid, const char *packet, int len) {
  gps_writebyte(0x10); // DLE
  if (packetid > 0xFF)
    gps_writebyte(packetid >> 8);
  gps_writebyte(packetid & 0xFF);
  for (int i = 0 ; i < len ; i++) {
    if (packet[i] == 0x10) // DLE escaping
      gps_writebyte(0x10);
    gps_writebyte(packet[i]);
  }
  gps_writebyte(0x10); // DLE
  gps_writebyte(0x03); // ETX
}

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

static void gps_set_utc_mode() {
  gps_write_tsip(0x8EA2,
    "\x03" // UTC time, UTC PPS
    , 1
  );
}

static void gps_set_primary_config() {
  gps_write_tsip(0xBB,
    "\x00" // Set config
    "\x07" // Overdetermined clock
    "\xff" // Reserved
    "\x04" // Stationary
    "\xff" // Reserved
//    "\x00\x00\x00\x3e" // Elevation mask: 10 degrees
    "\x00\x00\x80\x3d" // Elevation mask: 5 degrees
    "\x00\x00\x80\x40" // Signal mask: 4 AMU
//    "\x00\x00\x40\x41" // PDOP mask: 12
    "\x00\x00\x80\x41" // PDOP mask: 16
    "\x00\x00\xc0\x40" // 2D/3D switch: 6
    "\xff" // Reserved
    "\x02" // Foliage: always
    "\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff" // Reserved
    , 40
  );
}

static void gps_set_pps_config() {
  gps_write_tsip(0x8e4a,
    "\x01" // PPS on
    "\x00" // reserved
    "\x00" // rising PPS
    "\xbe\x64\xb1\x3d\xa9\x46\x9e\x6a"// cable delay = 38.5425ns
    "\x43\x96\x00\x00" // bias uncertainty threshold = 300m
    , 15
  );
}


static enum gps_state_t decoder_state = GPS_LEADER;
static char gps_message_valid;
static unsigned short gps_packetid;
static unsigned int gps_payload_len;
static unsigned char gps_payload[GPS_BUFFER_SIZE];
static unsigned char *gps_payload_ptr;
static char have_utcoffset = 0;

void gps_handle_message();

void gps_poll() {
  while (gps_can_read()) {
    int ch = gps_read();
    switch (decoder_state) {
      case GPS_LEADER:
        if (ch == 0x10)
          decoder_state = GPS_PACKETID;
        else {
          debug("Got garbage char "); debug_int(ch); debug(" from TSIP\r\n");
        }
        break;
      case GPS_PACKETID:
        gps_packetid = ch;
        gps_payload_len = 0;
        gps_payload_ptr = gps_payload;
        gps_message_valid = 1;
        if (ch == 0x8F) // "Superpacket" has 2-byte ID
          decoder_state = GPS_PACKETID2;
        else
          decoder_state = GPS_DATA;
        break;
      case GPS_PACKETID2:
        gps_packetid = (gps_packetid << 8) | ch;
        decoder_state = GPS_DATA;
        break;
      case GPS_DATA:
        if (ch == 0x10) 
          // Either the first part of an escaped DLE (DLE DLE), or end of packet (DLE ETX)
          decoder_state = GPS_DLE;
        else {
handle_data:
          gps_payload_len++;
          if (gps_payload_len > GPS_BUFFER_SIZE - 1)
            gps_message_valid = 0;
          else
            *(gps_payload_ptr++) = ch;
        }
        break;
      case GPS_DLE:
        if (ch == 0x10) { // DLE DLE = escaped DLE
          decoder_state = GPS_DATA;
          goto handle_data;
        } else if (ch == 0x03) { // DLE ETX = end of packet
          if (gps_message_valid)
            gps_handle_message();
          else {
            debug_int(gps_payload_len); debug(" byte payload too big\r\n");
          }
          decoder_state = GPS_LEADER;
        } else {
          debug("Unknown sequence DLE + "); debug_int(ch); debug(" from GPS\r\n");
          decoder_state = GPS_LEADER;
        }
    }
  }
}

void gps_timing_packet() {
  unsigned int gps_tow = (unsigned int)gps_payload[0] << 24
                | (unsigned int)gps_payload[1] << 16
                | (unsigned int)gps_payload[2] << 8
                | (unsigned int)gps_payload[3];
  unsigned short gps_week = (unsigned short)gps_payload[4] << 8
                | (unsigned short)gps_payload[5];
  signed short utc_offset = (signed short)gps_payload[6] << 8
                | (signed short)gps_payload[7];

  unsigned char timing_flag = gps_payload[8];

  unsigned char second = gps_payload[9];
  unsigned char minute = gps_payload[10];
  unsigned char hour = gps_payload[11];
  unsigned char day = gps_payload[12];
  unsigned char month = gps_payload[13];
  unsigned short year = (unsigned short)gps_payload[14] << 8
                | (unsigned short)gps_payload[15];


  debug_int(year); debug("-"); debug_int(month); debug("-"); debug_int(day);
  debug(" ");
  debug_int(hour); debug(":"); debug_int(minute); debug(":"); debug_int(second);

  static char *timing_flag_msg[][2] = {
    {"GPSTIME", "UTCTIME"},
    {"GPSPPS", "UTCPPS"},
    {"TIME", 0},
    {"UTCOFFSET", 0},
    {0, "TEST"}
  };

  for (int i = 0 ; i < sizeof(timing_flag_msg) / sizeof(*timing_flag_msg); i++) {
    const char *msg = timing_flag_msg[i][(timing_flag >> i) & 1];
    if (msg) {
      debug(" ");
      debug(msg);
    }
  }

  debug("\r\n");

  time_set_date(gps_week, gps_tow, -utc_offset);
  have_utcoffset = (timing_flag & 8) ? 0 : 1;
}

void gps_supplemental_timing_packet() {
  static char *rcv_mode_msg[] = {
    "AUTO", "1SAT", "MODE2", "2D", "3D", "DGPR", "CLOCK2D", "CLOCKOD"
  };

  static char *alarm_msg[] = {
    0, "ANT_OPEN", "ANT_SHORT", "NOT_TRACKING", 0, "SURVEYING",
    "NO_POSITION", "LEAP", "TEST_MODE", "BAD_POSITION",
    0, "ALMANAC"
  };

  static unsigned short alarm_mask = 0xBEE;

  static char *gps_status_msg[] = {
    "OK", "NO_TIME", 0, "PDOP", 0, 0, 0, 0,
    "0SV", "1SV", "2SV", "3SV", "BAD_SV", 0, 0, 0,
    "TRAIM_ERROR"
  };

  unsigned char rcv_mode = gps_payload[0];
  unsigned char survey_pct = gps_payload[2];
  unsigned short alarm = (unsigned short)gps_payload[9] << 8 
                       | (unsigned short)gps_payload[10];
  unsigned char gps_status = gps_payload[11];
  
  volatile float quantization_error;
  char *qeptr = (char *)(&quantization_error);
  memcpy(qeptr, gps_payload + 62, 1);
  memcpy(qeptr + 1, gps_payload + 61, 1);
  memcpy(qeptr + 2, gps_payload + 60, 1);
  memcpy(qeptr + 3, gps_payload + 59, 1); 
  int quantization_error_ns = nearbyint(quantization_error);
  time_set_sawtooth(quantization_error_ns);
  debug("GPS Mode: "); 
  if (rcv_mode < sizeof(rcv_mode_msg) / sizeof(*rcv_mode_msg)) {
    debug(rcv_mode_msg[rcv_mode]);
  } else {
    debug(rcv_mode);
  }
  debug(" Status: "); 
  if (gps_status < sizeof(gps_status_msg) / sizeof(*gps_status_msg)) {
    debug(gps_status_msg[gps_status]);
  } else {
    debug(gps_status);
  }

  if (alarm & alarm_mask) {
    debug(" Alarm:");
    for (int i = 0 ; i < sizeof(alarm_msg) / sizeof(*alarm_msg) ; i++) {
      if (alarm & 1 << i) {
        debug(" ");
        debug(alarm_msg[i]);
      }
    }
  }
  if (alarm & 0x20) {
    debug(" Survey: ");
    debug(survey_pct);
    debug("%");
  }

  debug("\r\n");

  enum gps_status_t status = GPS_OK;
  if (alarm & 0x820)
    status = GPS_MINOR_ALARM;
  if (alarm & 0x34e)
    status = GPS_UNLOCK;
  if (gps_status == 1 || gps_status == 8 || gps_status == 12 || gps_status == 16)
    status = GPS_UNLOCK;
  if (!have_utcoffset)
    status = GPS_UNLOCK;
  health_set_gps_status(status);
  health_reset_gps_watchdog();
}

void gps_handle_message() {
  switch (gps_packetid) {
    case 0x8FAB:
      gps_timing_packet();
      break;
    case 0x8FAC:
      gps_supplemental_timing_packet();
      break;
    default:
      debug("Got "); debug_int(gps_payload_len);
      debug(" byte message, type "); debug_int(gps_packetid);
      debug("\r\n");
      break;
  }
}

void gps_init() {
  GPS.begin(9600, SERIAL_8O1);;
  delay(100);
  gps_set_utc_mode();
  gps_set_pps_config();
}

#endif
