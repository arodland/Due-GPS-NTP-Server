#include "config.h"

void console_init() {
  while (!Serial);

  Console.begin(115200);
}
