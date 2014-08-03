#ifndef __DEBUG_H
#define __DEBUG_H

#if DEBUG

#define debug(x) Console.print(x)
#define debug_int debug
#define debug_long debug
#define debug_float(x) Console.print(x, 2)
#define debug_hex(x) Console.print(x, HEX)

#else

#define debug(x)
#define debug_int(x)
#define debug_long(x)
#define debug_float(x)
#define debug_hex(x)

#endif /* DEBUG */

#endif
