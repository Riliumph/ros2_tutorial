# devcontainerの使い方

## 開発環境として

VSCodeがssh-serverを立ててログインしてくれるので特にいうことはない。

## ROS Serverコンテナとして

開発環境と同じイメージをサーバー環境に採用。  
サーバー側でのROS実行を可能している。

### ROS Serverコンテナへの入り方

docker outside of dockerが採用されている前提とする。

```console
$ docker exec -it ros2_tutorial-ros_server-1 /bin/bash
```

#### composeコマンドは？

VSCodeのdevcontainerはcomposeコマンドは使えない

```console
$ docker compose ps -a
NAME      IMAGE     COMMAND   SERVICE   CREATED   STATUS    PORTS
```

composeがコンテナを知らない。  
コンテナを知らないから、動いていないと判断される。

```console
$ docker compose exec -it ros_server /bin/bash
service "ros_server" is not running
```
