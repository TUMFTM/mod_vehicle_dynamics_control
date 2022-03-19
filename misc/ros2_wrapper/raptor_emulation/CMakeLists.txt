cmake_minimum_required(VERSION 3.5)
project(tum_raptor_emulation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-w -fpermissive)
endif()

# Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(raptor_dbw_msgs REQUIRED)
find_package(deep_orange_msgs REQUIRED)
find_package(can_dbc_parser REQUIRED)
find_package(can_msgs REQUIRED)

# executable for raptor emulation
add_executable(tum_raptor_emulation
  src/RaptorEmulation.cpp)

ament_target_dependencies(tum_raptor_emulation
  "rclcpp"
  "can_dbc_parser"
  "std_msgs"
  "raptor_dbw_msgs"
  "deep_orange_msgs"
  "can_msgs")

# Install nodes
install(TARGETS
  tum_raptor_emulation
  DESTINATION lib/${PROJECT_NAME})

# ament package generation and installing
ament_auto_package(INSTALL_TO_SHARE params launch)
