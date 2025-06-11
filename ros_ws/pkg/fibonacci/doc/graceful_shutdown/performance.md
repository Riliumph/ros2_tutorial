# シャットダウンループのパフォーマンス

ループによるチェックを行う。

> 本当はイベントドリブンにするべき

```cpp
void
CheckGracefulShutdown(std::shared_ptr<fibonacci::ActionClient> cli)
{
  RCLCPP_INFO(rclcpp::get_logger(node_name), "start to check gs");
  keep_check_graceful_shutdown.store(true, std::memory_order_relaxed);
  while (keep_check_graceful_shutdown.load(std::memory_order_relaxed)) {
    if (graceful_shutdown.load(std::memory_order_relaxed)) {
      RCLCPP_INFO(rclcpp::get_logger(node_name), "graceful shutdown required");
      cli->Abort();
      RCLCPP_INFO(rclcpp::get_logger(node_name), "cancel completed");
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  RCLCPP_INFO(rclcpp::get_logger(node_name), "thread terminated");
}
```

## `std::this_thread::yield()`を使う

```console
$ pidstat -p 33105 1 10
Linux 6.6.87.1-microsoft-standard-WSL2 (devcontainer)   2025年06月11日  _x86_64_        (4 CPU)

19時26分52秒   UID       PID    %usr %system  %guest   %wait    %CPU   CPU  Command
19時26分53秒  1000     33105    8.00   92.00    0.00    0.00  100.00     1  fibonacci_actio
19時26分54秒  1000     33105   14.00   86.00    0.00    0.00  100.00     0  fibonacci_actio
19時26分55秒  1000     33105    8.00   92.00    0.00    0.00  100.00     0  fibonacci_actio
19時26分56秒  1000     33105   10.00   91.00    0.00    0.00  101.00     0  fibonacci_actio
19時26分57秒  1000     33105   14.00   85.00    0.00    0.00   99.00     2  fibonacci_actio
19時26分58秒  1000     33105    6.00   94.00    0.00    0.00  100.00     2  fibonacci_actio
19時26分59秒  1000     33105    7.00   94.00    0.00    0.00  101.00     2  fibonacci_actio
19時27分00秒  1000     33105    8.00   91.00    0.00    0.00   99.00     2  fibonacci_actio
19時27分01秒  1000     33105   12.00   89.00    0.00    0.00  101.00     1  fibonacci_actio
19時27分02秒  1000     33105    8.00   91.00    0.00    0.00   99.00     1  fibonacci_actio
Average:     1000     33105    9.50   90.50    0.00    0.00  100.00     -  fibonacci_actio
```

## スリープ 0sec

おおー。100%張り付いている。

```console
$ pidstat -p 26288 1 10
Linux 6.6.87.1-microsoft-standard-WSL2 (devcontainer)   2025年06月11日  _x86_64_        (4 CPU)

19時10分26秒   UID       PID    %usr %system  %guest   %wait    %CPU   CPU  Command
19時10分27秒  1000     26288  100.00    0.00    0.00    0.00  100.00     0  fibonacci_actio
19時10分28秒  1000     26288  100.00    0.00    0.00    0.00  100.00     2  fibonacci_actio
19時10分29秒  1000     26288  100.00    0.00    0.00    0.00  100.00     0  fibonacci_actio
19時10分30秒  1000     26288  100.00    0.00    0.00    0.00  100.00     2  fibonacci_actio
19時10分31秒  1000     26288   98.00    0.00    0.00    0.00   98.00     0  fibonacci_actio
19時10分32秒  1000     26288  101.00    0.00    0.00    0.00  101.00     1  fibonacci_actio
19時10分33秒  1000     26288  100.00    0.00    0.00    0.00  100.00     0  fibonacci_actio
19時10分34秒  1000     26288  100.00    0.00    0.00    0.00  100.00     0  fibonacci_actio
19時10分35秒  1000     26288  100.00    0.00    0.00    0.00  100.00     0  fibonacci_actio
19時10分36秒  1000     26288  100.00    0.00    0.00    0.00  100.00     3  fibonacci_actio
Average:     1000     26288   99.90    0.00    0.00    0.00   99.90     -  fibonacci_actio
```

## スリープ 1nsec

ナノ秒でだいぶCPU負荷が減った。

> 噂によると、`std::this_thread::sleep_for()`の精度はOS依存でマイクロ秒～ミリ秒とのこと。  
> 確かに、マイクロ秒と大差ないので、実はナノ秒スリープできていないのかも？

```console
$ pidstat -p 29295 1 10
Linux 6.6.87.1-microsoft-standard-WSL2 (devcontainer)   2025年06月11日  _x86_64_        (4 CPU)

19時17分59秒   UID       PID    %usr %system  %guest   %wait    %CPU   CPU  Command
19時18分00秒  1000     29295    3.00   10.00    0.00    0.00   13.00     0  fibonacci_actio
19時18分01秒  1000     29295    1.00    9.00    0.00    0.00   10.00     0  fibonacci_actio
19時18分02秒  1000     29295    2.00   10.00    0.00    0.00   12.00     0  fibonacci_actio
19時18分03秒  1000     29295    2.00    9.00    0.00    0.00   11.00     3  fibonacci_actio
19時18分04秒  1000     29295    3.00    9.00    0.00    0.00   12.00     3  fibonacci_actio
19時18分05秒  1000     29295    3.00    7.00    0.00    0.00   10.00     1  fibonacci_actio
19時18分06秒  1000     29295    1.00   11.00    0.00    0.00   12.00     1  fibonacci_actio
19時18分07秒  1000     29295    1.00   11.00    0.00    0.00   12.00     1  fibonacci_actio
19時18分08秒  1000     29295    3.00    9.00    0.00    0.00   12.00     3  fibonacci_actio
19時18分09秒  1000     29295    1.00   10.00    0.00    0.00   11.00     0  fibonacci_actio
Average:     1000     29295    2.00    9.50    0.00    0.00   11.50     -  fibonacci_actio
```

## スリープ 1usec

マイクロ秒はナノ秒と大差なし。

```console
$ pidstat -p 31200 1 10
Linux 6.6.87.1-microsoft-standard-WSL2 (devcontainer)   2025年06月11日  _x86_64_        (4 CPU)

19時20分41秒   UID       PID    %usr %system  %guest   %wait    %CPU   CPU  Command
19時20分42秒  1000     31200    3.00    8.00    0.00    0.00   11.00     0  fibonacci_actio
19時20分43秒  1000     31200    1.00    9.00    0.00    0.00   10.00     0  fibonacci_actio
19時20分44秒  1000     31200    1.00   11.00    0.00    0.00   12.00     1  fibonacci_actio
19時20分45秒  1000     31200    2.00    9.00    0.00    0.00   11.00     1  fibonacci_actio
19時20分46秒  1000     31200    2.00   10.00    0.00    0.00   12.00     3  fibonacci_actio
19時20分47秒  1000     31200    1.00   10.00    0.00    0.00   11.00     2  fibonacci_actio
19時20分48秒  1000     31200    0.00   10.00    0.00    0.00   10.00     2  fibonacci_actio
19時20分49秒  1000     31200    3.00    9.00    0.00    0.00   12.00     1  fibonacci_actio
19時20分50秒  1000     31200    1.00   10.00    0.00    0.00   11.00     1  fibonacci_actio
19時20分51秒  1000     31200    1.00   10.00    0.00    0.00   11.00     2  fibonacci_actio
Average:     1000     31200    1.50    9.60    0.00    0.00   11.10     -  fibonacci_actio
```

## スリープ 1msec

1msecでこんなにCPU空くのか。。。

```console
$ pidstat -p 28134 1 10
Linux 6.6.87.1-microsoft-standard-WSL2 (devcontainer)   2025年06月11日  _x86_64_        (4 CPU)

19時15分55秒   UID       PID    %usr %system  %guest   %wait    %CPU   CPU  Command
19時15分56秒  1000     28134    1.00    1.00    0.00    0.00    2.00     1  fibonacci_actio
19時15分57秒  1000     28134    0.00    1.00    0.00    0.00    1.00     1  fibonacci_actio
19時15分58秒  1000     28134    0.00    1.00    0.00    0.00    1.00     1  fibonacci_actio
19時15分59秒  1000     28134    0.00    1.00    0.00    0.00    1.00     1  fibonacci_actio
19時16分00秒  1000     28134    0.00    0.00    0.00    0.00    0.00     2  fibonacci_actio
19時16分01秒  1000     28134    0.00    1.00    0.00    0.00    1.00     2  fibonacci_actio
19時16分02秒  1000     28134    0.00    1.00    0.00    0.00    1.00     2  fibonacci_actio
19時16分03秒  1000     28134    0.00    1.00    0.00    0.00    1.00     2  fibonacci_actio
19時16分04秒  1000     28134    0.00    1.00    0.00    0.00    1.00     1  fibonacci_actio
19時16分05秒  1000     28134    1.00    1.00    0.00    0.00    2.00     2  fibonacci_actio
Average:     1000     28134    0.20    0.90    0.00    0.00    1.10     -  fibonacci_actio
```

## スリープ 1sec

とうとうCPU 0%になっちゃった。

```console
$ pidstat -p 34199 1 10
Linux 6.6.87.1-microsoft-standard-WSL2 (devcontainer)   2025年06月11日  _x86_64_        (4 CPU)

19時29分06秒   UID       PID    %usr %system  %guest   %wait    %CPU   CPU  Command
19時29分07秒  1000     34199    0.00    0.00    0.00    0.00    0.00     2  fibonacci_actio
19時29分08秒  1000     34199    0.00    0.00    0.00    0.00    0.00     1  fibonacci_actio
19時29分09秒  1000     34199    0.00    0.00    0.00    0.00    0.00     3  fibonacci_actio
19時29分10秒  1000     34199    0.00    1.00    0.00    0.00    1.00     1  fibonacci_actio
19時29分11秒  1000     34199    0.00    0.00    0.00    0.00    0.00     3  fibonacci_actio
19時29分12秒  1000     34199    0.00    0.00    0.00    0.00    0.00     3  fibonacci_actio
19時29分13秒  1000     34199    0.00    0.00    0.00    0.00    0.00     2  fibonacci_actio
19時29分14秒  1000     34199    0.00    0.00    0.00    0.00    0.00     1  fibonacci_actio
19時29分15秒  1000     34199    0.00    0.00    0.00    0.00    0.00     1  fibonacci_actio
19時29分16秒  1000     34199    0.00    0.00    0.00    0.00    0.00     1  fibonacci_actio
Average:     1000     34199    0.00    0.10    0.00    0.00    0.10     -  fibonacci_actio
```
