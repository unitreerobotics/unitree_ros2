cmake_minimum_required(VERSION 3.5)
project(unitree_ros2)

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

include_directories(include third_party/include)

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

SET(SRC_DIR ${PROJECT_SOURCE_DIR}/src)
AUX_SOURCE_DIRECTORY(${SRC_DIR}/client CLIENT_SRC)
AUX_SOURCE_DIRECTORY(${SRC_DIR}/client/g1 G1_CLIENT_SRC)
AUX_SOURCE_DIRECTORY(${SRC_DIR}/client/go2 GO2_CLIENT_SRC)
AUX_SOURCE_DIRECTORY(${SRC_DIR}/utils UTILS_SRC)

add_library(unitree_ros2 SHARED ${CLIENT_SRC} ${G1_CLIENT_SRC} ${GO2_CLIENT_SRC} ${UTILS_SRC})
ament_target_dependencies(unitree_ros2 ${DEPENDENCY_LIST})

install(TARGETS 
        unitree_ros2
        DESTINATION)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY third_party/include/ DESTINATION include/)

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

ament_export_include_directories(
  include
)

ament_export_libraries(
  unitree_ros2
)

ament_package()
