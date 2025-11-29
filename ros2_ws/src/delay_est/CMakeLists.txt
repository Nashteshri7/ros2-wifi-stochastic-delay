cmake_minimum_required(VERSION 3.8)
project(delay_est)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# add executables
add_executable(sender src/sender.cpp)
add_executable(receiver src/receiver.cpp)

add_executable(ping-pong_sender src/ping-pong_sender.cpp)
add_executable(ping-pong_receiver src/ping-pong_receiver.cpp)

add_executable(one-to-many_sender src/one-to-many_sender.cpp)
add_executable(one-to-many_receiver src/one-to-many_receiver.cpp)

add_executable(50-joints_sender src/50-joints_sender.cpp)
add_executable(50-joints_receiver src/50-joints_receiver.cpp)

add_executable(image_sender src/image_sender.cpp)
add_executable(image_receiver src/image_receiver.cpp)


ament_target_dependencies(sender rclcpp sensor_msgs)
ament_target_dependencies(receiver rclcpp sensor_msgs)

ament_target_dependencies(ping-pong_sender rclcpp sensor_msgs)
ament_target_dependencies(ping-pong_receiver rclcpp sensor_msgs)

ament_target_dependencies(one-to-many_sender rclcpp sensor_msgs)
ament_target_dependencies(one-to-many_receiver rclcpp sensor_msgs)

ament_target_dependencies(50-joints_sender rclcpp sensor_msgs)
ament_target_dependencies(50-joints_receiver rclcpp sensor_msgs)

ament_target_dependencies(image_sender rclcpp sensor_msgs)
ament_target_dependencies(image_receiver rclcpp sensor_msgs)

install(TARGETS
  sender
  receiver
  ping-pong_sender
  ping-pong_receiver
  one-to-many_sender
  one-to-many_receiver
  50-joints_sender
  50-joints_receiver
  image_sender
  image_receiver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
