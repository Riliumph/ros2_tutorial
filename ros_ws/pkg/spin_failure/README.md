# Spin Failure

`ctrl+c`で止まらない間違ったスピンを確認するパッケージ

## パッケージの作成方法

```console
$ ros2 pkg create --build-type ament_cmake --license MIT spin_failure
```

## ビルド方法

```console
$ colcon build --packages-up-to spin_failure
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
