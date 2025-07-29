cmake_minimum_required(VERSION 3.5)
project(unitree_ros2_example)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(include include/common include/nlohmann)
link_directories(src)

if($ENV{ROS_DISTRO} STREQUAL "humble")
    add_compile_definitions(ROS_DISTRO_HUMBLE)
elseif($ENV{ROS_DISTRO} STREQUAL "foxy")
    add_compile_definitions(ROS_DISTRO_FOXY)
endif()

if(ROS_DISTRO_HUMBLE)
    message(STATUS "Compiling for ROS 2 Humble")
elseif(ROS_DISTRO_FOXY)
    message(STATUS "Compiling for ROS 2 Foxy")
endif()

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
  rosbag2_cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_executable(low_level_ctrl src/low_level_ctrl.cpp src/common/motor_crc.cpp)
add_executable(low_level_ctrl_hg src/h1-2/lowlevel/low_level_ctrl_hg.cpp src/common/motor_crc_hg.cpp)
add_executable(g1_low_level_example src/g1/lowlevel/g1_low_level_example.cpp src/common/motor_crc_hg.cpp)
add_executable(read_low_state src/read_low_state.cpp)
add_executable(read_low_state_hg src/read_low_state_hg.cpp)
add_executable(read_motion_state src/read_motion_state.cpp)
add_executable(read_wireless_controller src/read_wireless_controller.cpp)
#add_executable(record_bag src/record_bag.cpp)
add_executable(go2_sport_client src/go2/go2_sport_client.cpp src/common/ros2_sport_client.cpp)
add_executable(go2_stand_example src/go2/go2_stand_example.cpp src/common/motor_crc.cpp)

if($ENV{ROS_DISTRO} STREQUAL "humble")
  add_executable(go2_robot_state_client src/go2/go2_robot_state_client.cpp src/common/motor_crc.cpp)
endif()

ament_target_dependencies(low_level_ctrl ${DEPENDENCY_LIST})
ament_target_dependencies(low_level_ctrl_hg ${DEPENDENCY_LIST})
ament_target_dependencies(g1_low_level_example ${DEPENDENCY_LIST})
ament_target_dependencies(read_low_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_low_state_hg ${DEPENDENCY_LIST})
ament_target_dependencies(read_motion_state ${DEPENDENCY_LIST})
ament_target_dependencies(read_wireless_controller ${DEPENDENCY_LIST})
#ament_target_dependencies(record_bag ${DEPENDENCY_LIST})
ament_target_dependencies(go2_sport_client ${DEPENDENCY_LIST})
ament_target_dependencies(go2_stand_example ${DEPENDENCY_LIST})

if($ENV{ROS_DISTRO} STREQUAL "humble")
  ament_target_dependencies(go2_robot_state_client ${DEPENDENCY_LIST})
endif()

install(TARGETS 
        low_level_ctrl
        low_level_ctrl_hg
        g1_low_level_example
        read_low_state
        read_low_state_hg
        read_motion_state
        read_wireless_controller
        go2_sport_client
        go2_stand_example
        DESTINATION)

if($ENV{ROS_DISTRO} STREQUAL "humble")
  install(TARGETS go2_robot_state_client)
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
