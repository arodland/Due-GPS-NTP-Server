#ifndef __CONSOLE_H
#define __CONSOLE_H

extern void console_init();
extern void console_handle_input();
extern void console_write_buffered();

extern char console_input;
extern char *console_outbuf;

#define in_interrupt() (__get_IPSR() & 0x1f)

#define console_print(...) do{\
  if (in_interrupt()) {\
    String _tmpstr(__VA_ARGS__);\
    strncat(console_outbuf, _tmpstr.c_str(), CONSOLE_OUTBUF_SIZE - (strlen(console_outbuf) + 1));\
  } else {\
    Console.print(__VA_ARGS__);\
  }\
} while(0)

#endif
