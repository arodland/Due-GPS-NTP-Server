#ifndef __DEBUG_H
#define __DEBUG_H

#define debug(x) SerialUSB.print(x)
#define debug_int debug
#define debug_long debug
#define debug_float(x) SerialUSB.print(x, 2)
#define debug_hex(x) SerialUSB.print(x, HEX)

#endif
