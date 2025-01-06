cmake_minimum_required(VERSION 3.8)
project(serial_port)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(arm_control REQUIRED)
find_package(serial_driver REQUIRED)

# 包含头文件目录
include_directories(
  include
  lib/vr_serial/include
)

# 添加共享库目录
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 添加可执行文件
add_executable(${PROJECT_NAME}_node src/serial_port.cpp)

# 指定依赖的库
target_link_libraries(${PROJECT_NAME}_node
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/vr_serial/libvr_serial.so
)

# 设置依赖的目标
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  arm_control
  serial_driver
)

# 安装可执行文件
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装共享库
install(FILES lib/vr_serial/libvr_serial.so
  DESTINATION lib/
)

# link_directories(/opt/ros/humble/lib)

ament_package()
