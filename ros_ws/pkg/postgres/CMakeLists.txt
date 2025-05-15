cmake_minimum_required(VERSION 3.8)
project(postgres)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
# pqxxライブラリの検出
find_library(PQ pq)
find_library(PQXX pqxx)

# 実行バイナリのビルド
set(EXE_NAME ${PROJECT_NAME}_node)
add_executable(${EXE_NAME}
  src/main.cpp
)
target_include_directories(${EXE_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  PRIVATE
    ${PQXX_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
)
target_link_libraries(${EXE_NAME}
  ${PQXX}
  ${PQ}
)
ament_target_dependencies(${EXE_NAME}
  rclcpp
)

# インストール
install(
  TARGETS ${EXE_NAME}
  DESTINATION lib/${PROJECT_NAME} # これ以外だとros2 runが見つけられない
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

ament_package()
