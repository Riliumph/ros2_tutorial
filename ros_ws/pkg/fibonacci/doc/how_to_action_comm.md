# アクション通信について

## サーバーの起動

```console
$ ros2 launch pkg/fibonacci/launch/server_launch.yaml
```

## アクション通信

> サーバー側はコンポーネント実装のため同じバイナリ

```console
$ ros2 run fibonacci fibonacci_action_client_node -- 5
```
