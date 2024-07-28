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
    |     ├ action_client.hpp
    |     ├ action_server.hpp
    |     ├ service_server.hpp
    |     └ visibility_control.h
    ├ lib/
    | ├ fibonacci/
    | | ├ fibonacci_action_client_node*
    | | └ fibonacci_server_component_node*
    | ├ libfibonacci_action_client.so
    | └ libfibonacci_server_component.so
    └ share/
```

## アクション通信実行方法

```console
$ ros2 run fibonacci fibonacci_action_server_node
[INFO] [1722048831.388800882] [fibonacci_action_server_node]: Received goal request with order 10
[INFO] [1722048831.389373052] [fibonacci_action_server_node]: Executing goal
[INFO] [1722048831.389648572] [fibonacci_action_server_node]: Publish feedback
[INFO] [1722048832.389698197] [fibonacci_action_server_node]: Publish feedback
[INFO] [1722048833.389715935] [fibonacci_action_server_node]: Publish feedback
[INFO] [1722048834.389649130] [fibonacci_action_server_node]: Publish feedback
[INFO] [1722048835.389685327] [fibonacci_action_server_node]: Publish feedback
[INFO] [1722048836.389709750] [fibonacci_action_server_node]: Publish feedback
[INFO] [1722048837.389657990] [fibonacci_action_server_node]: Publish feedback
[INFO] [1722048838.389709553] [fibonacci_action_server_node]: Publish feedback
[INFO] [1722048839.389713237] [fibonacci_action_server_node]: Publish feedback
[INFO] [1722048840.389877511] [fibonacci_action_server_node]: Goal succeeded
```

```console
$ ros2 run fibonacci fibonacci_action_client_node
[INFO] [1722048831.388202250] [fibonacci_action_client_node]: Sending goal
[INFO] [1722048831.389205866] [fibonacci_action_client_node]: Goal accepted by server, waiting for result
[INFO] [1722048831.389771564] [fibonacci_action_client_node]: Next number in sequence received: 0 1 1
[INFO] [1722048832.389847440] [fibonacci_action_client_node]: Next number in sequence received: 0 1 1 2
[INFO] [1722048833.389886039] [fibonacci_action_client_node]: Next number in sequence received: 0 1 1 2 3
[INFO] [1722048834.389800860] [fibonacci_action_client_node]: Next number in sequence received: 0 1 1 2 3 5
[INFO] [1722048835.389853749] [fibonacci_action_client_node]: Next number in sequence received: 0 1 1 2 3 5 8
[INFO] [1722048836.389883712] [fibonacci_action_client_node]: Next number in sequence received: 0 1 1 2 3 5 8 13
[INFO] [1722048837.389833380] [fibonacci_action_client_node]: Next number in sequence received: 0 1 1 2 3 5 8 13 21
[INFO] [1722048838.389881988] [fibonacci_action_client_node]: Next number in sequence received: 0 1 1 2 3 5 8 13 21 34
[INFO] [1722048839.389883720] [fibonacci_action_client_node]: Next number in sequence received: 0 1 1 2 3 5 8 13 21 34 55
[INFO] [1722048840.390125752] [fibonacci_action_client_node]: Result received: 0 1 1 2 3 5 8 13 21 34 55
```
