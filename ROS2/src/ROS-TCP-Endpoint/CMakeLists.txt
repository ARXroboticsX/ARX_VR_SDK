cmake_minimum_required(VERSION 3.8)
project(ros_tcp_endpoint)

find_package(ament_cmake REQUIRED)

# install(
#   PROGRAMS
#   ros_tcp_endpoint/default_server_endpoint.py
#   ros_tcp_endpoint/client.py
#   ros_tcp_endpoint/server.py
#   ros_tcp_endpoint/tcp_sender.py
#   ros_tcp_endpoint/communication.py
#   ros_tcp_endpoint/service.py
#   ros_tcp_endpoint/thread_pauser.py
#   ros_tcp_endpoint/publisher.py
#   ros_tcp_endpoint/unity_service.py
#   ros_tcp_endpoint/exceptions.py
#   ros_tcp_endpoint/subscriber.py
#   DESTINATION lib/${PROJECT_NAME}
# )
install(
  PROGRAMS
  ros_tcp_endpoint/default_server_endpoint.so
  ros_tcp_endpoint/client.so
  ros_tcp_endpoint/server.so
  ros_tcp_endpoint/tcp_sender.so
  ros_tcp_endpoint/communication.so
  ros_tcp_endpoint/service.so
  ros_tcp_endpoint/thread_pauser.so
  ros_tcp_endpoint/publisher.so
  ros_tcp_endpoint/unity_service.so
  ros_tcp_endpoint/exceptions.so
  ros_tcp_endpoint/subscriber.so
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
