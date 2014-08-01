#ifndef __GPS_H
#define __GPS_H

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

extern void gps_init();
extern void gps_poll();
extern void gps_set_sirf();
extern void gps_enable_dgps();

extern void gps_write_nmea(const char *);
extern void gps_write_sirf(const char *, int);

#endif
