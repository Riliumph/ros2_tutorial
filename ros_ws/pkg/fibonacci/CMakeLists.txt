cmake_minimum_required(VERSION 3.8)
project(fibonacci)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
# original package
find_package(fibonacci_msg REQUIRED)

# ----- libraries -----
# アクションサーバーライブラリ
set(SRV_LIB "fibonacci_server_component") # 接頭辞libは不要
add_library(${SRV_LIB} SHARED
  src/fibonacci/action_server.cpp
  src/fibonacci/service_server.cpp
)
target_include_directories(${SRV_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(${SRV_LIB}
  PUBLIC c_std_99 cxx_std_17 # Require C99 and C++17
)
target_compile_definitions(${SRV_LIB}
  PRIVATE "FIBONACCI_BUILDING_LIBRARY"
)
# 実行ファイルのコンパイルターゲットに依存パッケージの情報を追加する
# ライブラリパス等をコンパイラに渡すために必要（TARGET_LINK_LIBRARIES等を呼ぶと同様）
ament_target_dependencies(${SRV_LIB}
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "fibonacci_msg"
)

# アクションクライアントライブラリ
set(ACT_CLI_LIB "fibonacci_action_client") # 接頭辞libは不要
add_library(${ACT_CLI_LIB} SHARED
  src/fibonacci/action_client.cpp
  src/fibonacci_action_client_main.cpp
)
target_include_directories(${ACT_CLI_LIB} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(${ACT_CLI_LIB}
  PUBLIC c_std_99 cxx_std_17 # Require C99 and C++17
)
target_compile_definitions(${ACT_CLI_LIB}
  PRIVATE "FIBONACCI_BUILDING_LIBRARY"
)
ament_target_dependencies(${ACT_CLI_LIB}
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "fibonacci_msg"
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${SRV_LIB} ${ACT_CLI_LIB}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ----- binaries -----
# サーバーバイナリ
set(SRV_BIN "fibonacci_server_component_node")
add_executable(${SRV_BIN}
  src/server_component.cpp
)
# 同一パッケージのライブラリはこちらに書く
target_link_libraries(${SRV_BIN}
  ${SRV_LIB}
)
# 別パッケージのライブラリはこちらに書く
ament_target_dependencies(${SRV_BIN}
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "fibonacci_msg"
)
install(
  TARGETS ${SRV_BIN}
  DESTINATION lib/${PROJECT_NAME}
)

# アクションクライアントバイナリ
set(ACT_CLI_BIN "fibonacci_action_client_node")
rclcpp_components_register_node(${ACT_CLI_LIB}
  PLUGIN "fibonacci::FibonacciActionClient"
  EXECUTABLE ${ACT_CLI_BIN}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ヘッダファイルのディレクトリ情報を他パッケージが使えるようにCMakeへエクスポート
ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
# ライブラリのファイル情報を他パッケージが使えるようにCMakeへエクスポート
ament_export_libraries(
  ${SRV_LIB}
  ${ACT_CLI_LIB}
)
# パッケージの情報を他パッケージが使えるようにCMakeへエクスポート
ament_export_targets(
  export_${PROJECT_NAME}
)

# colconエコシステムのresource indexにパッケージ情報を登録
# ROS2コマンドがパッケージを認識できるようになる
ament_package()
