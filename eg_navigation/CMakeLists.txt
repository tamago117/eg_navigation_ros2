cmake_minimum_required(VERSION 3.5)
project(eg_navigation)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})

# テスト設定
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# includeディレクトリのインストール
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
)

# config, launch, urdfディレクトリのインストール
install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

# ビルド設定の便利関数
function(custom_executable target)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} "rclcpp"
                                      "std_msgs"
                                      "tf2_ros"
                                      "tf2")
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()


ament_export_include_directories(include)



ament_package()
