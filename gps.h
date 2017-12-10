#ifndef __GPS_H
#define __GPS_H

extern void gps_init();
extern void gps_poll();

extern bool gps_get_timestamp(int32_t *dest);

#endif
