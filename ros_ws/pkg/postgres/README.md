# fibonacci

フィボナッチ数を計算する機能。

## パッケージの作成

```console
$ ros2 pkg create fibonacci --license MIT
```

## ビルド方法

```console
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug --packages-up-to postgres
```

## 成果物の確認

```bash

```

## ライフサイクル起動

[ライフサイクルについて](./doc/how_to_lifecycling.md)

## 通信実行方法

- [サービス通信](./doc/how_to_service_comm.md)
- [アクション通信](./doc/how_to_action_comm.md)

## デバッグ方法

[デバッグ方法](./doc/how_to_debug.md)

## 参考

- [Writing an action server and client (C++)](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html#writing-an-action-server-and-client-c)
- [Writing a simple service and client (C++)](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html#writing-a-simple-service-and-client-c)
