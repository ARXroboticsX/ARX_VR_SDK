cmake_minimum_required(VERSION 3.8)
project(arm_control)

# 找到需要的依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 声明消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PosCmd.msg"
)

# 添加可执行文件
add_executable(pos_cmd_publisher src/pos_cmd_publisher.cpp)

# 链接依赖
ament_target_dependencies(pos_cmd_publisher rclcpp std_msgs)

# 为发布的库链接消息类型支持
rosidl_target_interfaces(pos_cmd_publisher ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 安装
install(TARGETS pos_cmd_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# 添加对接口的依赖
ament_export_dependencies(rosidl_default_runtime)

ament_package()

