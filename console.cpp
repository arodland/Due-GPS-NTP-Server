#include "config.h"
#include "system.h"
#include "timing.h"
#include "gps.h"

#define WORDS 10

char console_input = 0;

static char cmd_buf[CONSOLE_CMDLINE_SIZE];
static char *cmd_word[WORDS];
static char *bufpos;
static int cmd_words = 0;

char console_outbuf[CONSOLE_OUTBUF_SIZE];

static void console_reset_input();
void console_handle_input();
static void console_handle_command();

void console_init() {
  while (!Serial);

  console_reset_input();
  Console.begin(115200);
}

static void console_reset_input() {
  console_input = 0;
  memset(cmd_buf, 0, sizeof(cmd_buf));
  bufpos = cmd_buf;
  memset(cmd_word, 0, sizeof(cmd_word));
  cmd_words = 0;
}

void console_write_buffered() {
  if (console_outbuf[0] != '\0') {
    Console.print(console_outbuf);
    console_outbuf[0] = '\0';
  }
}

void console_handle_input() {
  if (!Console.available())
    return;
  char ch = Console.read();

  if (!console_input) {
    console_input = 1;
    Console.print("> ");
  }

  if (ch == '\r') {
    Console.print("\r\n");
    if (cmd_word[cmd_words] && cmd_words + 1 < WORDS)
      cmd_words++;
    console_handle_command();
    return;
  } else if (ch == '\n') {
    return;
  } else if (ch == '\b' || ch == 127) { // backspace
    Console.print("\b \b");
    *(--bufpos) = 0;
    if (cmd_word[cmd_words] > bufpos) {
      cmd_word[cmd_words] = NULL;
      cmd_words--;
    }
  } else if (ch == 3 || ch == '\e') { // ^C or escape
    Console.print("^C\r\n");
    console_reset_input();
    return;
  } else if (bufpos - cmd_buf < CONSOLE_CMDLINE_SIZE - 1) {
    Console.print(ch);
    if (ch == ' ') {
      *(bufpos++) = 0;
      if (cmd_word[cmd_words] && cmd_words + 1 < WORDS)
        cmd_words++;
    } else {
      if (!cmd_word[cmd_words])
        cmd_word[cmd_words] = bufpos;
      *(bufpos++) = ch;
    }
  }
}

#define commandmatch(pos, keyword) \
  ( cmd_words > (pos) && !strcmp(cmd_word[pos], keyword) )

#define get_set_int(pos, getter, setter) do {\
  if (cmd_words == (pos + 1))\
    setter(atoi(cmd_word[pos]));\
  else if (cmd_words == (pos))\
    Console.println(getter());\
  else\
    goto invalid;\
} while (0)

#define getset(pos, type, prefix, suffix) \
  get_set_##type(pos, prefix##_get_##suffix, prefix##_set_##suffix)

static void console_handle_command() {
  if (commandmatch(0, "reboot")) {
    system_reboot();
  }  else if (commandmatch(0, "pll")) {
    if (commandmatch(1, "min"))
      getset(2, int, pll, min);
    else if (commandmatch(1, "max"))
      getset(2, int, pll, max);
    else if (commandmatch(1, "factor"))
      getset(2, int, pll, factor);
    else if (commandmatch(1, "disable"))
      pll_set_enabled(false);
    else if (commandmatch(1, "enable"))
      pll_set_enabled(true);
    else goto invalid;
  } else if (commandmatch(0, "fll")) {
    if (commandmatch(1, "min"))
      getset(2, int, fll, min);
    else if (commandmatch(1, "max"))
      getset(2, int, fll, max);
    else if (commandmatch(1, "factor"))
      getset(2, int, fll, factor);
    else if (commandmatch(1, "coeff"))
      getset(2, int, fll, coeff);
    else goto invalid;
  } else if (commandmatch(0, "gps")) {
    if (commandmatch(1, "init"))
      gps_init();
    else goto invalid;
  } else {
    invalid:
    Console.print("Unknown command ");
    for (int i = 0 ; i < cmd_words ; i++) {
      Console.print("[");
      Console.print(cmd_word[i]);
      Console.print("]");
      if (i < cmd_words - 1)
        Console.print(" ");
    }
    Console.print("\r\n");
  }
  console_reset_input();
}
