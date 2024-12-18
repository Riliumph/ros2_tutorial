# greet

何かを通信するシンプルなサンプル。

## ビルド方法

```console
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-up-to greet
```

## 起動方法

### publisher / subscriberを同時に起動する

```console
$ ros2 launch pkg/greet/launch/launch.yaml
```

### publisher / subscriberを単独で起動する

ComposableNodeを前提としてlibビルドされているため`ros2 run`で起動することはできない。  
そこで、`rclcpp_components`からコンテナを起動して、起動したコンテナにライブラリをロードする手法をとる。

> `ros2 run`は実行バイナリをros2経由で起動する。

コンテナを起動する。

```console
$ ros2 run rclcpp_components component_container --ros-args --remap __node:=greet_container
```

別ターミナルで、起動したコンテナに対してノードを動的ロードする。

```console
$ ros2 component load /greet_container greet greet::Subscriber --node-name greet_composable_node
```

## 通信方法

ターミナルAでサブスクライバだけを起動する。

```console
$ ros2 run rclcpp_components component_container --ros-args --remap __node:=greet_container
[INFO] [1734006783.337105477] [greet_container]: Load Library: /workspaces/ros_ws/install/greet/lib/libgreet_subscriber.so
[INFO] [1734006783.338695717] [greet_container]: Found class: rclcpp_components::NodeFactoryTemplate<greet::Subscriber>
[INFO] [1734006783.338736494] [greet_container]: Instantiate class: rclcpp_components::NodeFactoryTemplate<greet::Subscriber>
[INFO] [1734006783.344644692] [greet_composable_node]: greet_composable_node created
[INFO] [1734007496.749077342] [greet_composable_node]: I heard: HELLO, WORLD\!
```

ターミナルBでコンテナにライブラリをロードする。

```console
$ ros2 component load /greet_container greet greet::Subscriber --node-name greet_composable_node
Loaded component 1 into '/greet_container' container node as '/greet_composable_node'
```

ターミナルBでトピック名を調べる。

```console
$ ros2 topic list
/greet_topic
```

`/greet_topic`に対して通信する。（命名の実装部分はソースコードを検索すること）

```console
$ $ ros2 topic pub --once /greet_topic std_msgs/msg/String "{data: 'HELLO, WORLD\!'}"

```

## 参考

- [Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
