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
```

### アクション通信

> サーバー側はコンポーネント実装のため同じバイナリ

```console
$ ros2 run fibonacci fibonacci_action_client_node -- 5
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
