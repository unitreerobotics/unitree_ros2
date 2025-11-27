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
find_package(Eigen3 REQUIRED)

add_executable(low_level_ctrl src/low_level_ctrl.cpp src/common/motor_crc.cpp)
add_executable(low_level_ctrl_hg src/h1-2/lowlevel/low_level_ctrl_hg.cpp src/common/motor_crc_hg.cpp)
add_executable(g1_low_level_example src/g1/lowlevel/g1_low_level_example.cpp src/common/motor_crc_hg.cpp)
add_executable(read_low_state src/read_low_state.cpp)
add_executable(read_low_state_hg src/read_low_state_hg.cpp)
add_executable(read_motion_state src/read_motion_state.cpp)
add_executable(read_wireless_controller src/read_wireless_controller.cpp)
#add_executable(record_bag src/record_bag.cpp)
add_executable(go2_sport_client src/go2/go2_sport_client.cpp src/common/ros2_sport_client.cpp)
target_compile_features(go2_sport_client PRIVATE cxx_std_20)

add_executable(go2_stand_example src/go2/go2_stand_example.cpp src/common/motor_crc.cpp)
target_compile_features(go2_stand_example PRIVATE cxx_std_20)

add_executable(go2_robot_state_client src/go2/go2_robot_state_client.cpp src/common/motor_crc.cpp) 
target_compile_features(go2_robot_state_client PRIVATE cxx_std_20)

add_executable(g1_arm_sdk_dds_example src/g1/high_level/g1_arm_sdk_dds_example.cpp src/common/motor_crc.cpp)
target_compile_features(g1_arm_sdk_dds_example PRIVATE cxx_std_20)

add_executable(g1_arm_action_example src/g1/high_level/g1_arm_action_example.cpp src/common/motor_crc.cpp)
target_compile_features(g1_arm_action_example PRIVATE cxx_std_20)

add_executable(g1_dex3_example src/g1/dex3/g1_dex3_example.cpp src/common/motor_crc.cpp)
target_compile_features(g1_dex3_example PRIVATE cxx_std_20)

add_executable(g1_loco_client_example src/g1/high_level/loco_client_example.cpp src/common/motor_crc.cpp)
target_compile_features(g1_loco_client_example PRIVATE cxx_std_20)

add_executable(g1_ankle_swing_example src/g1/lowlevel/g1_ankle_swing_example.cpp src/common/motor_crc_hg.cpp)
target_compile_features(g1_ankle_swing_example PRIVATE cxx_std_20)

add_executable(g1_audio_client_example src/g1/audio_client/g1_audio_client_example.cpp)
target_compile_features(g1_audio_client_example PRIVATE cxx_std_20)

add_executable(b2w_sport_client src/b2w/b2w_sport_client.cpp src/common/ros2_b2_sport_client.cpp)
target_compile_features(b2w_sport_client PRIVATE cxx_std_20)

add_executable(b2w_stand_example src/b2w/b2w_stand_example.cpp src/common/motor_crc.cpp)
target_compile_features(b2w_stand_example PRIVATE cxx_std_20)

add_executable(b2_stand_example src/b2/b2_stand_example.cpp src/common/motor_crc.cpp)
target_compile_features(b2_stand_example PRIVATE cxx_std_20)

add_executable(b2_sport_client src/b2/b2_sport_client.cpp src/common/ros2_b2_sport_client.cpp)
target_compile_features(b2_sport_client PRIVATE cxx_std_20)


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


ament_target_dependencies(go2_robot_state_client ${DEPENDENCY_LIST})
ament_target_dependencies(g1_arm_sdk_dds_example ${DEPENDENCY_LIST})
ament_target_dependencies(g1_arm_action_example ${DEPENDENCY_LIST})
ament_target_dependencies(g1_dex3_example ${DEPENDENCY_LIST})
target_include_directories(g1_dex3_example PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(g1_loco_client_example ${DEPENDENCY_LIST})
ament_target_dependencies(g1_ankle_swing_example ${DEPENDENCY_LIST})

ament_target_dependencies(g1_audio_client_example ${DEPENDENCY_LIST})

ament_target_dependencies(b2w_sport_client ${DEPENDENCY_LIST})
ament_target_dependencies(b2w_stand_example ${DEPENDENCY_LIST})

ament_target_dependencies(b2_sport_client ${DEPENDENCY_LIST})
ament_target_dependencies(b2_stand_example ${DEPENDENCY_LIST})

find_package(yaml-cpp QUIET)
if(yaml-cpp_FOUND)
    if (${yaml-cpp_VERSION} VERSION_GREATER_EQUAL "0.6")
        message(STATUS "Found yaml-cpp version ${yaml-cpp_VERSION}")
        add_executable(g1_dual_arm_example src/g1/lowlevel/g1_dual_arm_example.cpp src/common/motor_crc_hg.cpp)
        target_compile_features(g1_dual_arm_example PRIVATE cxx_std_20)
        ament_target_dependencies(g1_dual_arm_example yaml-cpp ${DEPENDENCY_LIST})
    else()
        message(STATUS "yaml-cpp version ${yaml-cpp_VERSION} is too old, skipping build of g1_dual_arm_example.")
    endif()
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


install(TARGETS go2_robot_state_client)
install(TARGETS g1_arm_sdk_dds_example)
install(TARGETS g1_arm_action_example)
install(TARGETS g1_dex3_example)
install(TARGETS g1_loco_client_example)
install(TARGETS g1_ankle_swing_example)

if(yaml-cpp_FOUND)
    if (${yaml-cpp_VERSION} VERSION_GREATER_EQUAL "0.6")
      install(TARGETS g1_dual_arm_example)
    endif()
endif()

install(TARGETS g1_audio_client_example)



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
