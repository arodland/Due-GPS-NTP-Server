#ifndef __DEBUG_H
#define __DEBUG_H

#if DEBUG

#include "console.h"

#define _cprint(...) do { if(!console_input) Console.print(__VA_ARGS__); } while(0)

#define debug(x) _cprint(x)
#define debug_int(x) _cprint(x)
#define debug_long(x) _cprint(x)
#define debug_float(x) _cprint(x, 2)
#define debug_hex(x) _cprint(x, HEX)

#else

#define debug(x)
#define debug_int(x)
#define debug_long(x)
#define debug_float(x)
#define debug_hex(x)

#endif /* DEBUG */

#endif
