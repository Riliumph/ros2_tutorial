# forkでrosバイナリを動かしてみる

`ros2 run`や`ros2 launch`ではなく、通常の`fork+execve`でプロセスを立ち上げてみる。

## ビルド方法

```console
$ make all
```

## 実行方法

```console
$ ./bin/fork /workspaces/ros_ws/install/fibonacci/lib/fibonacci/fibonacci_action_client_node
```
