/*
 * base.hをインクルード
 * command関数（引数にint型cmd）を作成
 * simLoop関数（引数にint型pause）を作成
 * main関数を作成
 */

#include "base.h"

static void command(int cmd)
{
  switch(cmd) {
    case 'a':
      printf("ok");
      break;
    case ' ':
      printf("\n");
      break;
  }
}

void simLoop(int pause)
{
  base_loop(pause);
  // ----------
  // draw関数

  // ----------
}

int main(int argc, char *argv[])
{
  fn.command = &command;
  base_main();
  // ----------
  // create関数

  // ----------
  finish_main(argc, argv);
  return 0;
}
