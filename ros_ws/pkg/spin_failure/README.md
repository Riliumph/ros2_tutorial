# Spin Failure

`ctrl+c`で止まらない間違ったスピンを確認するパッケージ

## パッケージの作成方法

```console
$ ros2 pkg create --build-type ament_cmake --license MIT spin_failure
```

## ビルド方法

```console
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-up-to spin_failure
```

## 実行方法

ROS2にパッケージを認識させる

```console
$ . install/setup.bash
```

実行する

```console
$ ros2 run spin_failure spin_failure
```

`ctrl+c`で`SIGINT`を発呼しても止まらないことを確認できる。  
このループ方法は正しくない。

```console
$ ros2 run spin_failure spin_failure_node
[INFO] [1733214657.045837744] [hello_ros_world_node]: waiting ...
[INFO] [1733214657.045867280] [hello_ros_world_node]: spining ...
^C[INFO] [1733214657.747147870] [rclcpp]: signal_handler(signum=2)
[INFO] [1733214658.046104147] [hello_ros_world_node]: waiting ...
[INFO] [1733214659.046216474] [hello_ros_world_node]: waiting ...
[INFO] [1733214660.046332825] [hello_ros_world_node]: waiting ...
```

## デバッグ方法

`ros2`経由で実行する。

```console
$ ros2 run spin_failure spin_failure
```

PIDを検索する。

> VSCodeの場合、プロセス名でもリアルタイムgrep出来るので別にやらなくてもいい

```console
$ ps a | grep spin
10735 pts/3    Sl+    0:00 /usr/bin/python3 /opt/ros/jazzy/bin/ros2 run spin_failure spin_failure_node
10738 pts/3    tl+    0:00 /workspaces/ros_ws/install/spin_failure/lib/spin_failure/spin_failure_node
12247 pts/6    S+     0:00 grep --color=auto spin
```

VSCodeのデバッグタブから`ROS: Attach`を選択して実行する。  
C++やPythonかを尋ねられるので、C++を選択する。  
プロセス名を尋ねられるので、以下の実行バイナリの方を選択する。

- `/workspaces/ros_ws/install/spin_failure/lib/spin_failure/spin_failure_node`

> ROS2プロセスを選んでもブレイクはされない。  
> `/usr/bin/python3 /opt/ros/jazzy/bin/ros2 run spin_failure spin_failure_node`  
> 上記プロセスはROS2のマスタープロセスであり、指定されたノードを回しているだけに過ぎない。
