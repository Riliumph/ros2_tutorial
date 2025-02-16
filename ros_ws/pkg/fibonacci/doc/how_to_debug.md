# デバッグ方法

## [launch編]((https://github.com/ms-iot/vscode-ros/blob/master/doc/debug-support.md#launch))

## [attach編](https://github.com/ms-iot/vscode-ros/blob/master/doc/debug-support.md#attaching-to-a-c-node)

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
選択候補になるプロセスは、ROS2プロセスとノードプロセスがあるが、ノードプロセス(以下のコマンド例のPID: 14758)を選択すること。  
ROS2プロセスにアタッチしてもROS2の管理処理を行っているだけで、独自コードは実行されていないのでブレイクされない。

```console
$ ps aux | grep fibonacci
ubuntu   14738  0.3  0.7 1099528 79660 pts/3   Sl+  07:16   0:00 /usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch pkg/fibonacci/launch/server_launch.yaml
ubuntu   14758  0.0  0.3 754564 32020 pts/3    Sl+  07:16   0:00 /opt/ros/jazzy/lib/rclcpp_components/component_container --ros-args -r __node:=fibonacci_server_composable_node -r __ns:=/
ubuntu   15874  0.0  0.0   4844  2160 pts/6    S+   07:18   0:00 grep --color=auto fibonacci
```

`FibonacciActionServer::execute`などにブレイクポイントを張り、以下のコマンドでクライアントを起動して通信する。

```console
$ ros2 run fibonacci fibonacci_action_client_node
```

ブレイクされれば完了。
