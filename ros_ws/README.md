# ros_ws

ROS2の開発ワークスペースのルートディレクトリ。

## ディレクトリツリー

```bash
ros_ws/
├ pkg/      # パッケージソース管理ディレクトリ
├ build/    # 自動生成（git管理外）
├ install/  # 自動生成（git管理外）
└ log/      # 自動生成（git管理外）
```

## ビルドシステム

ROS2にはROS1時代の`catkin`と同様にビルドシステムが存在する。

- `cmake`  
  一番基礎のビルドシステムだが、ROS2への認識などもすべて手で行う必要があり非現実的。
- `ament`  
  ROS2 ardentで使われていたROS2専用のビルドシステム。
- `colcon`  
  ROS2 bouncyやcrystalで採用されたビルドシステム。ROS2だけでなくGradleやPythonなどもサポートする。

このリポジトリでは`colcon`を採用していく。

### colconコマンド

- `colcon build`: パッケージのビルド
- `colcon test`: パッケージのテスト
- `colcon test-result`: テストの結果表示
- `colcon list`: パッケージの一覧表示

### ビルド方法

```console
# debug mode building
$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# release mode building
colcon build
```

#### デバッグ情報の確認

```console
$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
$ readelf -S install/<package>/lib/<package>/<executable> | grep -i debug
  [30] .debug_aranges    PROGBITS         0000000000000000  000290de
  [31] .debug_info       PROGBITS         0000000000000000  0002adbe
  [32] .debug_abbrev     PROGBITS         0000000000000000  0006af1d
  [33] .debug_line       PROGBITS         0000000000000000  0006c7dd
  [34] .debug_str        PROGBITS         0000000000000000  0007222e
  [35] .debug_line_str   PROGBITS         0000000000000000  000e15d0
  [36] .debug_rnglists   PROGBITS         0000000000000000  000e29f5
```

独自に`-g`を付ける必要が無い事が分かる。
