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
|   └ libfibonacci_action_server.so
|
└ install/
  └ fibonacci/
    ├ include/
    | └ fibonacci
    |   └ fibonacci
    |     ├ fibonacci_action_server.hpp
    |     └ visibility_control.h
    ├ lib/
    | └ libfibonacci_action_server.so
    └ share/
```
