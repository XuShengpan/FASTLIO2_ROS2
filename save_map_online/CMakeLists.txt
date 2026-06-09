cmake_minimum_required(VERSION 3.8)
project(save_map_online)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(OpenMP QUIET)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(./include/)


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

file(GLOB SRC_LIST "./src/*.cpp")

add_executable(save_map_online_node main.cpp ${SRC_LIST})
ament_target_dependencies(save_map_online_node rclcpp std_msgs tf2 tf2_ros nav_msgs sensor_msgs pcl_conversions geometry_msgs)
target_link_libraries(save_map_online_node
        yaml-cpp
        ${PCL_LIBRARIES}
)
install(TARGETS save_map_online_node DESTINATION lib/${PROJECT_NAME})

# 安装 launch 和 config 目录
install(DIRECTORY
        launch
        config
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
