# ros_ws

## 何のディレクトリか？

ROS2の開発ワークスペースのルートディレクトリである。  

## ビルドシステム

ROS2にはROS1時代の`catkin`と同様にビルドシステムが存在する。

- cmake  
  一番基礎のビルドシステムだが、ROS2への認識などもすべて手で行う必要があり非現実的。
- ament  
  ROS2 ardentで使われていたROS2専用のビルドシステム。
- colcon  
  ROS2 bouncyやcrystalで採用されたビルドシステム。ROS2だけでなくGradleやPythonなどもサポートする。

このリポジトリでは`colcon`を採用していく。

### colconコマンド

- `colcon build`: パッケージのビルド
- `colcon test`: パッケージのテスト
- `colcon test-result`: テストの結果表示
- `colcon list`: パッケージの一覧表示

## ディレクトリツリー

```bash
ros_ws/
├ pkg/      # パッケージソース管理ディレクトリ
├ build/    # 自動生成（git管理外）
├ log/      # 自動生成（git管理外）
└ install/  # 自動生成（git管理外）
```

### ワークスペースの作り方

```console
$ colcon build
Summary: 0 packages finished [0.08s]
```
