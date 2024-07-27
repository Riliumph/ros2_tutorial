# fibonacci_msg

`fibonacci`パッケージで用いるメッセージ。

## 参考

- [Writing an action server and client (C++)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html#writing-an-action-server-and-client-c)
- [Writing a simple service and client (C++)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html#writing-a-simple-service-and-client-c)

> [「ROS」の基礎とROS 2プログラミングの実践 ⾼瀬 英希(京都⼤学／JSTさきがけ)](http://lab3.kuis.kyoto-u.ac.jp/~takase/ros/4dashing.pdf)  
> 「ROS 2では，独⾃定義のmsg (srv, action)はノード実装のパッケージとは別にライブラリ化する」とのこと。
> ROS1のようにnodeのビルドに依存関係を持たせることで同パッケージ内にメッセージを定義することはできなくなった模様。

## パッケージの作成

```console
$ ros2 pkg create fibonacci_msg --license MIT
```

## ビルド方法

```console
$ cd ~/ros_ws
$ colcon build --packages-select fibonacci_msg
```

### 成果物の確認

```bash
ros_ws/
├ build/
| └ fibonacci_msg/
|
└ install/
  └ fibonacci_msg/
    ├ include/
    | └ fibonacci_msg/
    |   └ fibonacci_msg/
    |     ├ action/
    |     | ├ detail/       # ROSのメッセージの定義ファイル群
    |     | ├ fibonacci.h   # C用メッセージ代表ヘッダ
    |     | └ fibonacci.hpp # C++用メッセージ代表ヘッダ
    |     └ srv/
    |       ├ detail/       # ROSのメッセージの定義ファイル群
    |       ├ fibonacci.h   # C用メッセージ代表ヘッダ
    |       └ fibonacci.hpp # C++用メッセージ代表ヘッダ
    |
    ├ lib/
    └ share/
```

### メッセージファイルの確認

パッケージを認識させる。

```console
$ source install/local_setup.bash
$ echo $AMENT_PREFIX_PATH | tr ":" "\\n"
/workspaces/ros_ws/install/fibonacci
/opt/ros/jazzy
```

パッケージパスに入ったことを確認できる。

次に、生成したメッセージ定義の確認。

```console
$ ros2 interface show fibonacci_msg/action/Fibonacci
int32 order
---
int32[] sequence
---
int32[] partial_sequence
--------------------------------
$ ros2 interface show fibonacci_msg/srv/Fibonacci
int32 order
---
int32[] sequence
```
