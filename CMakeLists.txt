cmake_minimum_required(VERSION 3.16)
project(binosense_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)

# Generate custom messages and services - force C++ only
set(ROSIDL_GENERATORS "cpp" CACHE INTERNAL "" FORCE)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BinosenseData.msg"
  "msg/BinosenseImage.msg"
  "srv/SetMotorPosition.srv"
  "srv/SetMotorSpeed.srv"
  "srv/SetCameraExposure.srv"
  "srv/SetCameraWhiteBalance.srv"
  "srv/SetCameraGain.srv"
  "srv/SetVOR.srv"
  "srv/SetSV.srv"
  "srv/SetDepthControl.srv"
  "srv/SaveData.srv"
  "srv/SetImageResolution.srv"
  "srv/SetExtraModule.srv"
  "srv/GetMotorLimits.srv"
  "srv/SetNeckPosition.srv"
  DEPENDENCIES sensor_msgs std_msgs
)

# SDK path resolution order:
#   1. BE_SDK_DIR cmake variable (e.g. -DBE_SDK_DIR=/path/to/be_sdk)
#   2. BE_SDK_DIR environment variable
#   3. Local be_sdk directory next to this package
#   4. be_sdk as a parent directory (within the monorepo)
if(NOT BE_SDK_DIR)
  if(DEFINED ENV{BE_SDK_DIR})
    set(BE_SDK_DIR "$ENV{BE_SDK_DIR}")
  elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../be_sdk/include")
    set(BE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../be_sdk")
  elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../include")
    set(BE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")
  else()
    message(FATAL_ERROR
      "BinoSense SDK not found. Please do one of the following:\n"
      "  1. Set environment variable: export BE_SDK_DIR=/path/to/be_sdk\n"
      "  2. Pass cmake variable: -DBE_SDK_DIR=/path/to/be_sdk\n"
      "  3. Place be_sdk directory alongside this package: binosense_ros2/../be_sdk\n"
      "  4. Clone the SDK repo: cd .. && git clone https://github.com/BinoSense/be_sdk be_sdk"
    )
  endif()
endif()

set(BE_SDK_INCLUDE_DIR "${BE_SDK_DIR}/include")
set(BE_SDK_LIB_DIR "${BE_SDK_DIR}/lib/linux")

if(NOT EXISTS "${BE_SDK_INCLUDE_DIR}")
  message(FATAL_ERROR "SDK include directory not found: ${BE_SDK_INCLUDE_DIR}")
endif()
if(NOT EXISTS "${BE_SDK_LIB_DIR}/libbionic_eyes_c.so")
  message(FATAL_ERROR "SDK library not found: ${BE_SDK_LIB_DIR}/libbionic_eyes_c.so")
endif()

message(STATUS "Using BinoSense SDK: ${BE_SDK_DIR}")
message(STATUS "  Include: ${BE_SDK_INCLUDE_DIR}")
message(STATUS "  Library: ${BE_SDK_LIB_DIR}")

include_directories(
  include
  ${BE_SDK_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  /opt/ros/jazzy/include/cv_bridge
)

link_directories(${BE_SDK_LIB_DIR})

# Main driver
add_executable(binosense_driver
  src/binosense_driver_combined.cpp
)

ament_target_dependencies(binosense_driver
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  tf2
  tf2_ros
  rosidl_default_runtime
)

target_link_libraries(binosense_driver
  ${OpenCV_LIBRARIES}
  bionic_eyes_c
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
if(cpp_typesupport_target)
  target_link_libraries(binosense_driver "${cpp_typesupport_target}")
endif()

# Control example
add_executable(binosense_control_example
  examples/binosense_control_example.cpp
)

ament_target_dependencies(binosense_control_example
  rclcpp
  std_msgs
  geometry_msgs
  rosidl_default_runtime
)

if(cpp_typesupport_target)
  target_link_libraries(binosense_control_example "${cpp_typesupport_target}")
endif()

# Data recorder
add_executable(binosense_data_recorder
  examples/binosense_data_recorder.cpp
)

ament_target_dependencies(binosense_data_recorder
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  rosidl_default_runtime
)

target_link_libraries(binosense_data_recorder
  ${OpenCV_LIBRARIES}
)

if(cpp_typesupport_target)
  target_link_libraries(binosense_data_recorder "${cpp_typesupport_target}")
endif()

install(TARGETS
  binosense_driver
  binosense_control_example
  binosense_data_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  examples/simple_control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ${BE_SDK_LIB_DIR}/libbionic_eyes_c.so
  DESTINATION lib/
)

ament_package()
