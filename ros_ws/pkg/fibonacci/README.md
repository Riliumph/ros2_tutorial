# fibonacci

フィボナッチ数を計算する機能。

## パッケージの作成

```console
$ ros2 pkg create fibonacci --license MIT
```

## デバッグ方法

### [launch編]((https://github.com/ms-iot/vscode-ros/blob/master/doc/debug-support.md#launch))

### [attach編](https://github.com/ms-iot/vscode-ros/blob/master/doc/debug-support.md#attaching-to-a-c-node)

サーバーをデバッグすると仮定して、サーバープログラムを実行しておく。

```console
$ ros2 launch server_launch.yaml
```

VSCodeのデバッグ画面から`launch.json`で定義されている`ROS: Attach`を選択する。  
F5や実行ボタンからデバッグを実行すると、以下の選択を迫られる。

- C++
- Python

C++を選択すると、アタッチするプロセスを選択する画面になる。  
`fibonacci`などで検索し、アタッチしたいプロセス（`component_container`）を選択するとGDBが繋がる。  
`FibonacciActionServer::execute`などにブレイクポイントを張り、以下のコマンドでクライアントを起動して通信する。

```console
$ ros2 run fibonacci fibonacci_action_client_node
```

ブレイクされれば完了。

### 成果物の確認

> 独自のモノだけ記載。  
> ROSが自動で作成する成果物に関しては原則省略とする。

```bash
ros_ws/
├ build/
| └ fibonacci/
|   ├ fibonacci_action_client_node*
|   ├ fibonacci_service_client_node*
|   ├ fibonacci_server_component_node*
|   ├ libfibonacci_action_client.so
|   ├ libfibonacci_action_server.so
|   ├ libfibonacci_service_client.so
|   └ libfibonacci_service_server.so
|
└ install/
  └ fibonacci/
    ├ include/
    | └ fibonacci/
    |   └ fibonacci/
    |     ├ action_client.hpp
    |     ├ action_server.hpp
    |     ├ service_client.hpp
    |     ├ service_server.hpp
    |     └ visibility_control.h
    ├ lib/
    | ├ fibonacci/
    | | ├ fibonacci_action_client_node*
    | | ├ fibonacci_service_client_node*
    | | └ fibonacci_server_component_node*
    | ├ libfibonacci_action_client.so
    | ├ libfibonacci_action_server.so
    | ├ libfibonacci_service_client.so
    | └ libfibonacci_service_server.so
    └ share/
```

## 通信実行方法

### サービス通信

```console
$ ros2 run fibonacci fibonacci_server_component_node
```

```console
$ ros2 run fibonacci fibonacci_service_client_node
[INFO] [1722153225.881993842] [fibonacci_service_client_node]: Sending request
```

### アクション通信

> サーバー側はコンポーネント実装のため同じバイナリ

```console
$ ros2 run fibonacci fibonacci_server_component_node
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
