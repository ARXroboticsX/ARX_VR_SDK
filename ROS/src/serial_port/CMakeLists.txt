cmake_minimum_required(VERSION 3.0.2)
project(serial_port)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  pos_cmd_msg
  serial
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES serial_port
 CATKIN_DEPENDS roscpp rospy std_msgs pos_cmd_msg
 DEPENDS system_lib
)


include_directories(
  include
  lib/vr_serial/include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)


add_executable(${PROJECT_NAME} src/serial_port.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC lib/vr_serial/include)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/vr_serial/libvr_serial.so)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES lib/vr_serial/libvr_serial.so
  DESTINATION lib/
)



