# ros2_tutorial

## 環境へのログイン

### devcontainer

devcontainer機能を持つIDEなら仮想環境内にログインできる。  
例としてVisual Studio Codeで記載する。  

- Dev Containers Extensionから、`reopen container`を選択

### 一般的なログイン

```console
$ docker compose up -d
$ docker compose exec -it devcontainer /bin/bash
```

> ただし、vimやemacsといったエディタしか使えないので注意  

## ROS2のdemo実行

標準インストールされている以下のパッケージを実行する。

- `demo_nodes_py listener`
- `demo_nodes_cpp talker`

```console
$ ros2 run demo_nodes_py listener
（wait to receive from talker）
[INFO] [1717837672.974251757] [talker]: Publishing: 'Hello World: 1'
[INFO] [1717837673.974250958] [talker]: Publishing: 'Hello World: 2'
... ... ...
... ... ...
```

```console
$ ros2 run demo_nodes_cpp talker
[INFO] [1717837672.974251757] [talker]: Publishing: 'Hello World: 1'
[INFO] [1717837673.974250958] [talker]: Publishing: 'Hello World: 2'
... ... ...
... ... ...
```

### 標準デモのインストール場所

```bash
/opt/ros/jazzy/
├ share/
| ├ demo_nodes_py
| | ├ package.xml
| | └ params.yaml
| └ demo_nodes_cpp
|   ├ package.xml
|   └ params.yaml
|
└ lib/
  ├ demo_nodes_py
  | ├ talker*
  | ├ listener*
  | └ etc...
  └ demo_nodes_cpp
    ├ talker*
    ├ listener*
    └ etc...
```

`listener`と`talker`はC++とPythonの両方で実装されている。  
ROS1からの伝統的なサンプルである`add_two_ints`など、色々なサンプルが揃っている。

## 参考

- [ROS2 Documentation: Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html)
- [WSL2による ROS 2 開発環境](https://qiita.com/kccs_yoshiteru-imamura/items/66155f9e54e2a86ec212#nvidia%E3%83%89%E3%83%A9%E3%82%A4%E3%83%90%E3%82%A4%E3%83%B3%E3%82%B9%E3%83%88%E3%83%BC%E3%83%AB)
