# ライフサイクルについて

ライフサイクルは以下である。

| ID  | 遷移 (Transition)       | 説明 |
|----|------------------|------------------------------|
| 1  | `configure`     | `unconfigured` → `inactive` |
| 2  | `transition_failure` | `configure` 失敗時の遷移 |
| 3  | `activate`      | `inactive` → `active` |
| 4  | `transition_failure` | `activate` 失敗時の遷移 |
| 5  | `deactivate`    | `active` → `inactive` |
| 6  | `transition_failure` | `deactivate` 失敗時の遷移 |
| 7  | `cleanup`       | `inactive` → `unconfigured` |
| 8  | `transition_failure` | `cleanup` 失敗時の遷移 |
| 9  | `shutdown`      | `unconfigured` → `finalized` |
| 10 | `transition_failure` | `shutdown` 失敗時の遷移 |

## コマンド

### 状態を取得するコマンド

```console
$ ros2 lifecycle get /fibonacci_action_server
```

### 状態を変更するコマンド

```console
$ ros2 service call /fibonacci_action_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: ${STATUS}}}"
```

## configure遷移

今の状態を確認する。

```console
$ ros2 lifecycle get /fibonacci_action_server
unconfigured [1]
```

状態を`unconfigured`から`inactive`に遷移させるため、`configure`トランジションを行う。

```console
$ ros2 service call /fibonacci_action_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
```

アクションサーバー側でログが出力される。

```console
[component_container-1] [INFO] [1738516582.480921778] [fibonacci_action_server]: on_configure() is called.
[component_container-1] [INFO] [1738516582.481012712] [fibonacci_action_server]: Establish Server
```

まだ通信はできない。

> フィボナッチサーバーの特殊仕様として`Receive`メソッドで拒否を返している。

```console
$ ros2 run fibonacci fibonacci_action_client_node -- 5
（省略）
---
[component_container-1] [INFO] [1738519129.959962487] [fibonacci_action_server]: [d01c72af36d51d4e42323bdca6f2c875] Receive goal request
[component_container-1] [ERROR] [1738519129.960019836] [fibonacci_action_server]: [d01c72af36d51d4e42323bdca6f2c875] Server is not active
```

> この状態でサーバーインスタンスは立ち上げるべきではないと思われる。  
> というのもアクションサーバー自体はクライアント側が`wait_for_action_server()`で待つことを期待する。  
> そのため、`wait_for_action_server()`は`true`を返すのに、リクエストが`Server is not active`で拒否されるのは意図が通じない。  
> 本来は`on_activate()`でサーバーインスタンスを作るべきだろう。  
> このサンプルコードでは、状態管理とアクセス拒否を実装したかったため、このような作りとしている。

## activate遷移

```console
$ ros2 service call /fibonacci_action_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
requester: making request: lifecycle_msgs.srv.ChangeState_Request(transition=lifecycle_msgs.msg.Transition(id=3, label=''))

response:
lifecycle_msgs.srv.ChangeState_Response(success=True)
```

```console
[component_container-1] [INFO] [1738519410.815729940] [fibonacci_action_server]: on_activate() is called.
```

この状態になると通信が可能となる。

```console
$ ros2 run fibonacci fibonacci_action_client_node -- 5
[INFO] [1738519448.820604831] [rclcpp]: プロセスID: 40645
[INFO] [1738519449.175039765] [fibonacci_action_client_node]: fibonacci_action_client_node created
[INFO] [1738519449.175158462] [fibonacci_action_client_node]: Sending request: 5
[INFO] [1738519449.175349626] [fibonacci_action_client_node]: Waiting for accept
[INFO] [1738519449.176296863] [fibonacci_action_client_node]: Request was accepted
---
[component_container-1] [INFO] [1738519449.175411003] [fibonacci_action_server]: [0606466624b8c3a5cc2ddf9f1fb98] Receive goal request
[component_container-1] [INFO] [1738519449.175476538] [fibonacci_action_server]: [0606466624b8c3a5cc2ddf9f1fb98] Request was accepted
[component_container-1] [INFO] [1738519449.175833209] [fibonacci_action_server]: [0606466624b8c3a5cc2ddf9f1fb98] Start execution of goal
[component_container-1] [INFO] [1738519449.176120787] [fibonacci_action_server]: [0606466624b8c3a5cc2ddf9f1fb98] === NEW REQUEST RECEIVED ===
[component_container-1] [INFO] [1738519449.176141717] [fibonacci_action_server]: [0606466624b8c3a5cc2ddf9f1fb98] Executing fibonacci service
（省略）
```
