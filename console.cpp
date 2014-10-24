#include "config.h"
#include "system.h"

#define BUFSIZE 512
#define WORDS 10

char console_input = 0;

static char cmd_buf[BUFSIZE + 1];
static char *cmd_word[WORDS];
static char *bufpos;
static int cmd_words = 0;

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

void console_handle_input() {
  if (!Console.available())
    return;
  char ch = Console.read();

  if (!console_input) {
    console_input = 1;
    Console.print("> ");
  }
  Console.print(ch);

  if (ch == '\r') {
    console_handle_command();
    return;
  } else if (ch == '\n') {
    return;
  } else if (ch == '\b' || ch == 127) { // backspace
    *(--bufpos) = 0;
    if (cmd_word[cmd_words] > bufpos) {
      cmd_word[cmd_words] = NULL;
      cmd_words--;
    }
  } else if (ch == 3 || ch == '\e') { // ^C or escape
    console_reset_input();
    return;
  } else if (bufpos - cmd_buf < BUFSIZE) {
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

static void console_handle_command() {
  if (!strcmp(cmd_word[0], "reboot")) {
    system_reboot();
  } else {
    invalid:
    Console.print("Unknown command ");
    for (int i = 0 ; i <= cmd_words ; i++) {
      Console.print("[");
      Console.print(cmd_word[i]);
      Console.print("]");
      if (i < cmd_words)
        Console.print(" ");
      else 
        Console.print("\r\n");
    }
  }
  console_reset_input();
}
