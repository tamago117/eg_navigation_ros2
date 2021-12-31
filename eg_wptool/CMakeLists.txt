cmake_minimum_required(VERSION 3.5)
project(eg_wptool)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})

# ビルド設定の便利関数
function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} "rclcpp")
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

# talkerノードのビルド設定
custom_executable(talker)
# listenerノードのビルド設定
custom_executable(listener)
custom_executable(set_and_get_parameter)
#custom_executable(lifecycle_talker)
add_executable(lifecycle_talker
  src/lifecycle_talker.cpp)
target_link_libraries(lifecycle_talker
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  lifecycle_talker
  DESTINATION lib/${PROJECT_NAME})

#launch
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
