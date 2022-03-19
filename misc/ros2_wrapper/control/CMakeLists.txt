cmake_minimum_required(VERSION 3.5)
project(tum_tmpc_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-w -fpermissive)
endif()

# Package dependencies
# Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tum_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(raptor_dbw_msgs REQUIRED)
find_package(deep_orange_msgs REQUIRED)
find_package(can_dbc_parser REQUIRED)
find_package(can_msgs REQUIRED)


# find all auto-generated files for controller only
file(GLOB_RECURSE CONTROLLER_ONLY_FILES src/controller_dev_py/build/slprj/*.cpp)

# gather all includes for auto-generaetd files for controller only
FILE(GLOB_RECURSE new_list_control src/controller_dev_py/*.h)
SET(dir_list_control "")
FOREACH(file_path_control ${new_list_control})
    GET_FILENAME_COMPONENT(dir_path_control ${file_path_control} PATH)
    SET(dir_list_control ${dir_list_control} ${dir_path_control})
ENDFOREACH()
LIST(REMOVE_DUPLICATES dir_list_control)

# executable for controller only
add_executable(control
  src/main.cpp
  src/ControlHandler.cpp
  src/dbw_state_machine.cpp
  src/controller_dev_py/build/controller_dev_py_ert_rtw/controller_dev_py.cpp
  src/controller_dev_py/build/controller_dev_py_ert_rtw/controller_dev_py_capi.cpp
  src/controller_dev_py/build/controller_dev_py_ert_rtw/controller_dev_py_data.cpp
  ${CONTROLLER_ONLY_FILES}
  ../../../lib/osqp/osqp_wrapper.c)

# includes for controller only
target_include_directories(control PRIVATE ${dir_list_control})
target_include_directories(control PRIVATE ../../../lib/osqp/)

# add libraries for controller only
target_link_directories(control PRIVATE ../../../lib/osqp/lib)
target_link_libraries(control osqp)

# add define for control
target_compile_definitions(control PRIVATE CONTROL)

ament_target_dependencies(control
  "rclcpp"
  "std_msgs"
  "tum_msgs"
  "novatel_oem7_msgs"
  "raptor_dbw_msgs"
  "deep_orange_msgs"
  "can_dbc_parser"
  "can_msgs")

# find all auto-generated files for sil
file(GLOB_RECURSE SIL_FILES src/trajectory_planning_dev_py/build/slprj/*.cpp)

# gather all includes for auto-generaetd files for sil
FILE(GLOB_RECURSE new_list_sil src/trajectory_planning_dev_py/*.h)
SET(dir_list_sil "")
FOREACH(file_path_sil ${new_list_sil})
    GET_FILENAME_COMPONENT(dir_path_sil ${file_path_sil} PATH)
    SET(dir_list_sil ${dir_list_sil} ${dir_path_sil})
ENDFOREACH()
LIST(REMOVE_DUPLICATES dir_list_sil)

# executable for SIL
add_executable(sil
   src/main.cpp
   src/ControlHandler.cpp
   src/dbw_state_machine.cpp
   src/trajectory_planning_dev_py/build/trajectory_planning_dev_py_ert_rtw/trajectory_planning_dev_py.cpp
   src/trajectory_planning_dev_py/build/trajectory_planning_dev_py_ert_rtw/trajectory_planning_dev_py_capi.cpp
   src/trajectory_planning_dev_py/build/trajectory_planning_dev_py_ert_rtw/trajectory_planning_dev_py_data.cpp
   ${SIL_FILES}
   ../../../lib/osqp/osqp_wrapper.c)

# includes for sil
target_include_directories(sil PRIVATE ${dir_list_sil})
target_include_directories(sil PRIVATE ../../../lib/osqp/)

# add libraries for sil
target_link_directories(sil PRIVATE ../../../lib/osqp/lib)
target_link_libraries(sil osqp)

# add define for SIL
target_compile_definitions(sil PRIVATE SIL)

ament_target_dependencies(sil
  "rclcpp"
  "std_msgs"
  "tum_msgs"
  "novatel_oem7_msgs"
  "raptor_dbw_msgs"
  "deep_orange_msgs"
  "can_dbc_parser"
  "can_msgs")

add_executable(tum_rt_logger src/tum_rt_logger.cpp)

ament_target_dependencies(tum_rt_logger "rclcpp" "std_msgs")

# Install nodes
install(TARGETS
  sil
  control
  tum_rt_logger
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_auto_package(INSTALL_TO_SHARE params launch)
