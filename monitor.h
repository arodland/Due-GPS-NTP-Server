#ifndef _MONITOR_H
#define _MONITOR_H

extern void monitor_send(const char *metric, int value);
extern void monitor_sendf(const char *metric, float value);
extern void monitor_flush();

#endif
