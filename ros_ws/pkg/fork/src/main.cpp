#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>
// system
#include <fcntl.h>
#include <pthread.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

extern char** environ;

int
main(int argc, char** argv)
{
  if (argc < 2) {
    printf("Usage: %s <command>\n", argv[0]);
    return -1;
  }
  printf("PID: %d\n", getgid());
  auto pid = fork();
  if (pid == -1) {
    perror("fork");
    return -1;
  }
  if (pid == 0) {
    printf("forked child process: %d\n", getpid());
    // 子プロセス
    // 子プロセスはfdの状態も引き継いでしまう。
    // 子プロセス側の処理でfdをすべて閉じる
    static const int start_fd = 3;
    for (auto i = start_fd; i < FD_SETSIZE; ++i) {
      close(i);
    }
    static const int sleep_time = 1;
    printf("sleep %d sec for attach gdb\n", sleep_time);
    sleep(sleep_time); // Attach用の時間
    auto sub_argc = 0;
    char* sub_argv[1024];
    for (int i = 1; i < argc; ++i) {
      sub_argv[sub_argc++] = argv[i];
    }
    sub_argv[sub_argc++] = NULL; // NULL終端
    printf("execve %s\n", sub_argv[0]);
    execve(sub_argv[0], sub_argv, environ);
    // 以降、到達の場合はエラー
    printf("error: %s", strerror(errno));
    _exit(-1);
  }

  // 子プロセスの終了待機
  int status = 0;
  wait(&status);
  /* 終了ステータスのチェック */
  if (WIFEXITED(status)) {
    printf("親プロセス : 子プロセスは終了ステータス%dで正常終了しました\n",
           WEXITSTATUS(status));
  }
  if (WIFSIGNALED(status)) {
    printf("親プロセス : 子プロセスはシグナル番号%dで終了しました\n",
           WTERMSIG(status));
  }
  return 0;
}
