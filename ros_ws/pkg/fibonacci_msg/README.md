# fibonacci_msg

`fibonacci`パッケージで用いるメッセージ。

> [「ROS」の基礎とROS 2プログラミングの実践 ⾼瀬 英希(京都⼤学／JSTさきがけ)](http://lab3.kuis.kyoto-u.ac.jp/~takase/ros/4dashing.pdf)  
> 「ROS 2では，独⾃定義のmsg (srv, action)はノード実装のパッケージとは別にライブラリ化する」とのこと。
> ROS1のようにnodeのビルドに依存関係を持たせることで同パッケージ内にメッセージを定義することはできなくなった模様。

## パッケージの作成

```console
$ ros2 pkg create fibonacci_msg --license MIT
```
