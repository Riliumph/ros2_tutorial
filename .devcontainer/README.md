# devcontainerの使い方

## 開発環境として

VSCodeがssh-serverを立ててログインしてくれるので特にいうことはない。

## ROS Serverコンテナとして

開発環境と同じイメージをサーバー環境に採用。  
サーバー側でのROS実行を可能している。

### ROS Serverコンテナへの入り方

docker-in-dockerが採用されている前提とする。

```console
$ docker compose exec -it ros_server /bin/bash
```
