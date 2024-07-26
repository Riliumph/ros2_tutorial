# fibonacci

フィボナッチ数を計算する機能。

## マイルストーン

アクション通信のサポート実施。  
今後、サービス通信をサポート予定。

## パッケージの作成

```console
$ ros2 pkg create fibonacci --library-name fibonacci_action_server --license MIT
```

## ビルド方法

### デバッグビルド

```console
$ cd ~/ros_ws
$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-select fibonacci
```

### リリースビルド

```console
$ cd ~/ros_ws
$ colcon build --packages-select fibonacci_action
```

### 成果物の確認

> 独自のモノだけ記載。  
> ROSが自動で作成する成果物に関しては原則省略とする。

```bash
ros_ws/
├ build/
| └ fibonacci/
|   ├ fibonacci_action_client_node
|   ├ fibonacci_action_server_node
|   ├ libfibonacci_action_client.so
|   └ libfibonacci_action_server.so
|
└ install/
  └ fibonacci/
    ├ include/
    | └ fibonacci/
    |   └ fibonacci/
    |     ├ fibonacci_action_client.hpp
    |     ├ fibonacci_action_server.hpp
    |     └ visibility_control.h
    ├ lib/
    | ├ fibonacci/
    | | ├ fibonacci_action_client_node*
    | | └ fibonacci_action_server_node*
    | ├ libfibonacci_action_client.so
    | └ libfibonacci_action_server.so
    └ share/
```

## 実行方法

```console
$ ros2 run fibonacci fibonacci_action_server_node
[INFO] [1721889430.549764550] [fibonacci_action_server_node]: Received goal request with order 10
[INFO] [1721889430.550338732] [fibonacci_action_server_node]: Executing goal
[INFO] [1721889430.550635394] [fibonacci_action_server_node]: Publish feedback
[INFO] [1721889431.550644811] [fibonacci_action_server_node]: Publish feedback
```

```console
$ ros2 run fibonacci fibonacci_action_client_node
[INFO] [1721889430.549137257] [fibonacci_action_client_node]: Sending goal
[INFO] [1721889430.550193397] [fibonacci_action_client_node]: Goal accepted by server, waiting for result
[INFO] [1721889430.550754700] [fibonacci_action_client_node]: Next number in sequence received: 0 1 1
[INFO] [1721889431.550796228] [fibonacci_action_client_node]: Next number in sequence received: 0 1 1 2
[INFO] [1721889432.550810725] [fibonacci_action_client_node]: Next number in sequence received: 0 1 1 2 3
```
