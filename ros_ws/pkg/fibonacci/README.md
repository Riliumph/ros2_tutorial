# fibonacci

フィボナッチ数を計算する機能。

## パッケージの作成

```console
$ ros2 pkg create fibonacci --license MIT
```

## 成果物の確認

> 独自のモノだけ記載。  
> ROSが自動で作成する成果物に関しては原則省略とする。

```bash
ros_ws/
├ build/
| └ fibonacci/
|   ├ fibonacci_action_client_node*
|   ├ fibonacci_service_client_node*
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
    | | └ fibonacci_service_client_node*
    | ├ libfibonacci_action_client.so
    | ├ libfibonacci_action_server.so
    | ├ libfibonacci_service_client.so
    | └ libfibonacci_service_server.so
    └ share/
```

## 通信実行方法

### サーバーの起動

```console
$ ros2 launch server_launch.yaml
```

### サービス通信

```console
$ ros2 run fibonacci fibonacci_service_client_node
[INFO] [1722153225.881993842] [fibonacci_service_client_node]: Sending request
```

### アクション通信

> サーバー側はコンポーネント実装のため同じバイナリ

```console
$ ros2 run fibonacci fibonacci_action_client_node -- 5
[INFO] [1725045200.881663577] [rclcpp]: プロセスID: 85577
[INFO] [1725045200.896723363] [fibonacci_action_client_node]: Sending request: 5
[INFO] [1725045200.896952644] [fibonacci_action_client_node]: Waiting for accept
[INFO] [1725045200.897489030] [fibonacci_action_client_node]: request was accepted
[INFO] [1725045200.897512123] [fibonacci_action_client_node]: Request result
[INFO] [1725045200.897596181] [fibonacci_action_client_node]: Wait for result
[INFO] [1725045200.897945930] [fibonacci_action_client_node]: Received feedback
[INFO] [1725045200.897981157] [fibonacci_action_client_node]: Next number in sequence received: 0 1 1
[INFO] [1725045201.898061268] [fibonacci_action_client_node]: Received feedback
[INFO] [1725045201.898116993] [fibonacci_action_client_node]: Next number in sequence received: 0 1 1 2
[INFO] [1725045202.898045685] [fibonacci_action_client_node]: Received feedback
[INFO] [1725045202.898107632] [fibonacci_action_client_node]: Next number in sequence received: 0 1 1 2 3
[INFO] [1725045203.898066105] [fibonacci_action_client_node]: Received feedback
[INFO] [1725045203.898120307] [fibonacci_action_client_node]: Next number in sequence received: 0 1 1 2 3 5
[INFO] [1725045204.898333474] [fibonacci_action_client_node]: request was accepted
[INFO] [1725045204.898394689] [fibonacci_action_client_node]: request was succeeded
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
