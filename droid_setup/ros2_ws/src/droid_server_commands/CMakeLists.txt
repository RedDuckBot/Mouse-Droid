cmake_minimum_required(VERSION 3.8)
project(droid_server_commands)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(droid_interfaces REQUIRED)

find_package(fmt REQUIRED)

include_directories(
  include
)

add_executable(droid_server src/droid_server.cpp src/SoundPlayer.cpp 
  src/DYPlayer.cpp  src/MotorDirection.cpp src/Device.cpp src/DigitalOutput.cpp 
  src/MotorDriver.cpp src/PullDirection.cpp src/Switch.cpp src/RPi_BNO055.cpp 
  src/PID.cpp)

#Link libraries
target_link_libraries(droid_server pigpio fmt pthread)

#Link ROS2 packages
ament_target_dependencies(droid_server rclcpp rclcpp_action droid_interfaces)

install(TARGETS 
    droid_server
    DESTINATION lib/${PROJECT_NAME}/    
)

ament_package()