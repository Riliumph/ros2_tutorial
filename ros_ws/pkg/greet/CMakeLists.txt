cmake_minimum_required(VERSION 3.8)
project(greet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
# original package
find_package(greet_msg REQUIRED)

# ----- libraries -----
# publisher
set(PUB_LIB "greet_publisher") # 接頭辞libは不要
add_library(${PUB_LIB} SHARED
  src/greet/publisher.cpp
)
target_include_directories(${PUB_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(${PUB_LIB}
  PUBLIC c_std_99 cxx_std_17 # Require C99 and C++17
)
target_compile_definitions(${PUB_LIB}
  # visibility_control.h用
  PRIVATE "GREET_BUILDING_LIBRARY"
)
ament_target_dependencies(${PUB_LIB}
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "greet_msg"
)
rclcpp_components_register_nodes(${PUB_LIB} "greet::Publisher")

# subscriber
set(SUB_LIB "greet_subscriber") # 接頭辞libは不要
add_library(${SUB_LIB} SHARED
  src/greet/subscriber.cpp
)
target_include_directories(${SUB_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(${SUB_LIB}
  PUBLIC c_std_99 cxx_std_17 # Require C99 and C++17
)
target_compile_definitions(${SUB_LIB}
  PRIVATE "GREET_BUILDING_LIBRARY"
)
ament_target_dependencies(${SUB_LIB}
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "greet_msg"
)
rclcpp_components_register_nodes(${SUB_LIB} "greet::Subscriber")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PUB_LIB} ${SUB_LIB}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
