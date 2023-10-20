cmake_minimum_required(VERSION 3.5)
project(unitree_go)

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

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_dds_idl REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/AudioData.msg"
        "msg/BmsCmd.msg"
        "msg/BmsState.msg"
        "msg/Error.msg"
        "msg/Go2FrontVideoData.msg"
        "msg/HeightMap.msg"
        "msg/IMUState.msg"
        "msg/InterfaceConfig.msg"
        "msg/LidarState.msg"
        "msg/LowCmd.msg"
        "msg/LowState.msg"
        "msg/MotorCmd.msg"
        "msg/MotorState.msg"
        "msg/PathPoint.msg"
        "msg/Req.msg"
        "msg/Res.msg"
        "msg/SportModeCmd.msg"
        "msg/SportModeState.msg"
        "msg/TimeSpec.msg"
        "msg/UwbState.msg"
        "msg/UwbSwitch.msg"
        "msg/WirelessController.msg"
  DEPENDENCIES geometry_msgs
)

rosidl_generate_dds_interfaces(
  ${rosidl_generate_interfaces_TARGET}__dds_connext_idl
  IDL_TUPLES ${rosidl_generate_interfaces_IDL_TUPLES}
  OUTPUT_SUBFOLDERS "dds_connext"
)
add_dependencies(
  ${PROJECT_NAME}
  ${PROJECT_NAME}__dds_connext_idl
)



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
