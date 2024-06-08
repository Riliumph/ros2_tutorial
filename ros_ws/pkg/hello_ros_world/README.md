# Hello, ROS World

## パッケージの作成方法

ROSのワークスペースに移動する。

```console
$ cd ros_ws
```

パッケージの作成コマンドを投入する。

```console
$ ros2 pkg create --build-type ament_cmake --license MIT hello_ros_world
```

## ビルド方法

```console
 $ colcon build
Starting >>> hello_ros_world
Finished <<< hello_ros_world [4.53s]

Summary: 1 package finished [4.68s]
```

## 実行方法

ROS2にパッケージを認識させる

```console
$ . install/setup.bash
```

実行する

```console
$ ros2 run hello_ros_world hello_ros_world_node
```

### エラー

```console
$ colcon build
Starting >>> hello_ros_world
[0.152s] WARNING:colcon.colcon_ros.prefix_path.ament:The path '/workspaces/ros_ws/install/hello_ros_world' in the environment variable AMENT_PREFIX_PATH doesn't exist
[0.152s] WARNING:colcon.colcon_ros.prefix_path.catkin:The path '/workspaces/ros_ws/install/hello_ros_world' in the environment variable CMAKE_PREFIX_PATH doesn't exist
Finished <<< hello_ros_world [3.55s]

Summary: 1 package finished [3.63s]
```

このエラーは`install/setup.bash`を実行していないと発生する。
