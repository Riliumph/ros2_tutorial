cmake_minimum_required(VERSION 3.8)
project(fibonacci)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
# original package
find_package(fibonacci_msg REQUIRED)

# ----- libraries -----
# サービスサーバー
set(SRV_SRV_LIB "fibonacci_service_server") # 接頭辞libは不要
add_library(${SRV_SRV_LIB} SHARED
  src/fibonacci/service_server.cpp
)
target_include_directories(${SRV_SRV_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(${SRV_SRV_LIB}
  PUBLIC c_std_99 cxx_std_17 # Require C99 and C++17
)
target_compile_definitions(${SRV_SRV_LIB}
  PRIVATE "FIBONACCI_BUILDING_LIBRARY"
)
ament_target_dependencies(${SRV_SRV_LIB}
  "rclcpp"
  "rclcpp_lifecycle"
  "rclcpp_action"
  "rclcpp_components"
  "fibonacci_msg"
)
rclcpp_components_register_nodes(${SRV_SRV_LIB} "fibonacci::ServiceServer")

# アクションサーバー
set(ACT_SRV_LIB "fibonacci_action_server") # 接頭辞libは不要
add_library(${ACT_SRV_LIB} SHARED
  src/fibonacci/action_server.cpp
)
target_include_directories(${ACT_SRV_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(${ACT_SRV_LIB}
  PUBLIC c_std_99 cxx_std_17 # Require C99 and C++17
)
target_compile_definitions(${ACT_SRV_LIB}
  PRIVATE "FIBONACCI_BUILDING_LIBRARY"
)
ament_target_dependencies(${ACT_SRV_LIB}
  "rclcpp"
  "rclcpp_lifecycle"
  "rclcpp_action"
  "rclcpp_components"
  "fibonacci_msg"
)
rclcpp_components_register_nodes(${ACT_SRV_LIB} "fibonacci::ActionServer")

# アクションクライアントライブラリ
set(ACT_CLI_LIB "fibonacci_action_client") # 接頭辞libは不要
add_library(${ACT_CLI_LIB} SHARED
  src/fibonacci/action_client.cpp
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
  "rclcpp_lifecycle"
  "rclcpp_action"
  "rclcpp_components"
  "fibonacci_msg"
)

# サービスクライアントライブラリ
set(SRV_CLI_LIB "fibonacci_service_client") # 接頭辞libは不要
add_library(${SRV_CLI_LIB} SHARED
  src/fibonacci/service_client.cpp
)
target_include_directories(${SRV_CLI_LIB} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(${SRV_CLI_LIB}
  PUBLIC c_std_99 cxx_std_17 # Require C99 and C++17
)
target_compile_definitions(${SRV_CLI_LIB}
  PRIVATE "FIBONACCI_BUILDING_LIBRARY"
)
ament_target_dependencies(${SRV_CLI_LIB}
  "rclcpp"
  "rclcpp_lifecycle"
  "rclcpp_components"
  "fibonacci_msg"
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${ACT_SRV_LIB} ${SRV_SRV_LIB} ${ACT_CLI_LIB} ${SRV_CLI_LIB}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ----- binaries -----
# アクションクライアントバイナリ
set(ACT_CLI_BIN "fibonacci_action_client_node")
add_executable(${ACT_CLI_BIN}
  src/action_client_main.cpp
  src/graceful_shutdown/flags.cpp
)
target_link_libraries(${ACT_CLI_BIN}
  ${ACT_CLI_LIB}
)
target_include_directories(${ACT_CLI_BIN} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${ACT_CLI_BIN}
  "rclcpp"
  "rclcpp_lifecycle"
  "rclcpp_action"
  "rclcpp_components"
  "fibonacci_msg"
)
# サービスクライアントバイナリ
set(SRV_CLI_BIN "fibonacci_service_client_node")
# rclcpp::spinとrclcpp::spin_until_future_completeの多重spinが発生するためNG
# 詳細はwikiへ
# rclcpp_components_register_node(${SRV_CLI_LIB}
#   PLUGIN "fibonacci::ServiceClient"
#   EXECUTABLE ${SRV_CLI_BIN}
# )
add_executable(${SRV_CLI_BIN}
  src/service_client_main.cpp
)
target_link_libraries(${SRV_CLI_BIN}
  ${SRV_CLI_LIB}
)
target_include_directories(${SRV_CLI_BIN} PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# 別パッケージのライブラリはこちらに書く
ament_target_dependencies(${SRV_CLI_BIN}
  "rclcpp"
  "rclcpp_lifecycle"
  "rclcpp_action"
  "rclcpp_components"
  "fibonacci_msg"
)
install(
  TARGETS ${SRV_CLI_BIN} ${ACT_CLI_BIN}
  DESTINATION lib/${PROJECT_NAME}
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

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)

ament_export_libraries(
  ${SRV_SRV_LIB}
  ${ACT_SRV_LIB}
  ${SRV_CLI_LIB}
  ${ACT_CLI_LIB}
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
