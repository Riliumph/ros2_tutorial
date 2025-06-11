# Graceful Shutdownする方法

## やり方

### サーバーの起動

サーバーを起動する

```console
$ ros2 launch pkg/fibonacci/launch/server_launch.yaml
```

サーバーの活性化させる。

```console
$ ros2 service call /fibonacci_action_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
$ ros2 service call /fibonacci_action_server/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
```

### クライアントの起動とSIGTERM

```console
$ ros2 run fibonacci fibonacci_action_client_node -- 8
[INFO] [1743451086.403616998] [your_rclcpp]: プロセスID: 40862
[INFO] [1743451086.423925640] [fibonacci_action_client_node]: fibonacci_action_client_node created
[INFO] [1743451086.424105413] [your_rclcpp]: start to check gs
[INFO] [1743451086.424110243] [fibonacci_action_client_node]: Sending request: 8
[INFO] [1743451086.424375840] [fibonacci_action_client_node]: Waiting for accept
[INFO] [1743451086.425603573] [fibonacci_action_client_node]: [NULL] Connection was accepted
[INFO] [1743451086.425646155] [fibonacci_action_client_node]: [2b8636baaf81babdd3503ac3fc54ccb0] Request was accepted
[INFO] [1743451086.425657737] [fibonacci_action_client_node]: [2b8636baaf81babdd3503ac3fc54ccb0] Request result
[INFO] [1743451086.425730716] [fibonacci_action_client_node]: [2b8636baaf81babdd3503ac3fc54ccb0] Waiting for result
[INFO] [1743451086.425914597] [fibonacci_action_client_node]: [2b8636baaf81babdd3503ac3fc54ccb0] Received feedback
[INFO] [1743451086.425937190] [fibonacci_action_client_node]: [2b8636baaf81babdd3503ac3fc54ccb0] Next number in sequence received: 0 1 1 
[INFO] [1743451087.425964258] [fibonacci_action_client_node]: [2b8636baaf81babdd3503ac3fc54ccb0] Received feedback
[INFO] [1743451087.426018171] [fibonacci_action_client_node]: [2b8636baaf81babdd3503ac3fc54ccb0] Next number in sequence received: 0 1 1 2 
[INFO] [1743451088.425971337] [fibonacci_action_client_node]: [2b8636baaf81babdd3503ac3fc54ccb0] Received feedback
[INFO] [1743451088.426025381] [fibonacci_action_client_node]: [2b8636baaf81babdd3503ac3fc54ccb0] Next number in sequence received: 0 1 1 2 3 
[INFO] [1743451089.425954222] [fibonacci_action_client_node]: [2b8636baaf81babdd3503ac3fc54ccb0] Received feedback
[INFO] [1743451089.426008375] [fibonacci_action_client_node]: [2b8636baaf81babdd3503ac3fc54ccb0] Next number in sequence received: 0 1 1 2 3 5 
[INFO] [1743451090.426002128] [fibonacci_action_client_node]: [2b8636baaf81babdd3503ac3fc54ccb0] Received feedback
[INFO] [1743451090.426070188] [fibonacci_action_client_node]: [2b8636baaf81babdd3503ac3fc54ccb0] Next number in sequence received: 0 1 1 2 3 5 8 
```

クライアントを停止させる。

```console
$ kill 40862
```

```console
$ ros2 run fibonacci fibonacci_action_client_node -- 8
（省略）
[INFO] [1743451090.597550435] [rclcpp]: signal_handler(signum=15)
[INFO] [1743451090.597601722] [your_rclcpp]: Caught signal 15
[ERROR] [1743451090.597856118] [fibonacci_action_client_node]: [2b8636baaf81babdd3503ac3fc54ccb0] Request was interrupted
[INFO] [1743451090.597887027] [your_rclcpp]: Request was aborted
[INFO] [1743451090.597951690] [your_rclcpp]: joining gs_checker
[INFO] [1743451090.627636417] [your_rclcpp]: thread terminated
[INFO] [1743451090.627806000] [fibonacci_action_client_node]: fibonacci_action_client_node finalized
```

### 注意

アクションサーバーからの`Result`を待っている際に`SIGTERM`が送られるケースが多い。  
この場合、以下の関数で使われている`rclcpp::spin_until_future_complete()`が `rclcpp::FutureReturnCode::INTERRUPTED`の値を即座に返す。

このことは[C++ ROS Client Library API Document](https://docs.ros2.org/latest/api/rclcpp/namespacerclcpp.html#a7b4ff5f1e516740d7e11ea97fe6f5532)に記載されている。

> Return codes to be used with spin_until_future_complete.
>
> SUCCESS: The future is complete and can be accessed with "get" without blocking. This does not indicate that the operation succeeded; "get" may still throw an exception.  
> INTERRUPTED: The future is not complete, spinning was interrupted by Ctrl-C or another error.  
> TIMEOUT: Spinning timed out.

「スピンがインタラプトされたら」とあるので、シグナルはROSも受け取っている事が分かる。仕組みはわからない。

```cpp
ActionClient::GoalHandle::WrappedResult
ActionClient::ReceiveResponse(GoalHandle::SharedPtr goal_handle)
{
  auto gid = goal_handle->get_goal_id();
  REQ_INFO(gid, "Request result");
  auto result_future = client->async_get_result(goal_handle);
  REQ_INFO(gid, "Waiting for result");
  auto response = rclcpp::spin_until_future_complete(get_node_base_interface(),
                                                     result_future);
  GoalHandle::WrappedResult interrupted_result;
  interrupted_result.goal_id = gid;
  interrupted_result.code = rclcpp_action::ResultCode::ABORTED;
  switch (response) {
    case rclcpp::FutureReturnCode::SUCCESS:
      REQ_INFO(gid, "Request was succeeded");
      break;
    case rclcpp::FutureReturnCode::INTERRUPTED:
      REQ_ERROR(gid, "Request was interrupted");
      return interrupted_result;
    case rclcpp::FutureReturnCode::TIMEOUT:
      REQ_ERROR(gid, "Request was timeout");
      return interrupted_result;
    default:
      REQ_ERROR(gid, "Request was missed");
      return interrupted_result;
  }
  return result_future.get();
}
```
