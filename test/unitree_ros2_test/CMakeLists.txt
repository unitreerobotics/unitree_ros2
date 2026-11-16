cmake_minimum_required(VERSION 3.16)
project(unitree_ros2_test C CXX)
find_package(ament_cmake REQUIRED)
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  include(cmake/dds_peer.cmake)
  find_package(rclcpp REQUIRED)
  add_executable(legacy_header_compat_test legacy_header_compat_test.cpp)
  target_compile_features(legacy_header_compat_test PRIVATE cxx_std_17)
  target_include_directories(legacy_header_compat_test PRIVATE
    "../../example/src/include" "../../example/src/include/common")
  ament_target_dependencies(legacy_header_compat_test rclcpp unitree_api)
  add_executable(generic_header_compat_test generic_header_compat_test.cpp)
  target_compile_features(generic_header_compat_test PRIVATE cxx_std_17)
  target_include_directories(generic_header_compat_test PRIVATE
    "../../example/src/include" "../../example/src/include/common"
    "../../example/src/include/nlohmann")
  ament_target_dependencies(generic_header_compat_test rclcpp unitree_api)
  add_executable(api_client_test api_client_test.cpp)
  target_compile_features(api_client_test PRIVATE cxx_std_17)
  target_include_directories(api_client_test PRIVATE
    "../../example/src/include" "../../example/src/include/common")
  ament_target_dependencies(api_client_test rclcpp unitree_api)
  ament_add_pytest_test(api_client test_api_client.py TIMEOUT 30
    ENV "UNITREE_API_CLIENT_TEST=$<TARGET_FILE:api_client_test>")
  add_executable(api_client_peer_test api_client_peer_test.cpp)
  target_compile_features(api_client_peer_test PRIVATE cxx_std_17)
  target_include_directories(api_client_peer_test PRIVATE
    "../../example/src/include" "../../example/src/include/common")
  ament_target_dependencies(api_client_peer_test rclcpp unitree_api)
  add_executable(b2_api_client_peer_test api_client_peer_test.cpp)
  target_compile_definitions(b2_api_client_peer_test PRIVATE UNITREE_B2_CLIENT)
  target_compile_features(b2_api_client_peer_test PRIVATE cxx_std_17)
  target_include_directories(b2_api_client_peer_test PRIVATE
    "../../example/src/include" "../../example/src/include/common")
  ament_target_dependencies(b2_api_client_peer_test rclcpp unitree_api)
  ament_add_pytest_test(api_client_peer test_api_client_peer.py TIMEOUT 180
    ENV "UNITREE_DDS_API_SERVER=$<TARGET_FILE:dds_api_server>"
    "UNITREE_API_CLIENT_PEER_TEST=$<TARGET_FILE:api_client_peer_test>"
    "UNITREE_B2_API_CLIENT_PEER_TEST=$<TARGET_FILE:b2_api_client_peer_test>")
  add_executable(b2_api_client_test b2_api_client_test.cpp)
  target_compile_features(b2_api_client_test PRIVATE cxx_std_17)
  target_include_directories(b2_api_client_test PRIVATE
    "../../example/src/include" "../../example/src/include/common")
  ament_target_dependencies(b2_api_client_test rclcpp unitree_api)
  ament_add_pytest_test(b2_api_client test_b2_api_client.py TIMEOUT 30
    ENV "UNITREE_B2_API_CLIENT_TEST=$<TARGET_FILE:b2_api_client_test>")
  foreach(variant unmatched_response_test b2_unmatched_response_test)
    add_executable(${variant} unmatched_response_test.cpp)
    target_compile_features(${variant} PRIVATE cxx_std_17)
    target_include_directories(${variant} PRIVATE
      "../../example/src/include" "../../example/src/include/common")
    ament_target_dependencies(${variant} rclcpp unitree_api)
  endforeach()
  target_compile_definitions(b2_unmatched_response_test PRIVATE UNITREE_B2_CLIENT)
  ament_add_pytest_test(unmatched_response test_unmatched_response.py TIMEOUT 60
    ENV "UNITREE_UNMATCHED_RESPONSE_TEST=$<TARGET_FILE:unmatched_response_test>"
    "UNITREE_B2_UNMATCHED_RESPONSE_TEST=$<TARGET_FILE:b2_unmatched_response_test>")
  foreach(kind api b2)
    add_executable(${kind}_deadline_test api_deadline_test.cpp)
    target_compile_features(${kind}_deadline_test PRIVATE cxx_std_17)
    target_include_directories(${kind}_deadline_test PRIVATE "../../example/src/include")
    ament_target_dependencies(${kind}_deadline_test rclcpp unitree_api)
  endforeach()
  target_compile_definitions(b2_deadline_test PRIVATE UNITREE_B2_CLIENT)
  ament_add_pytest_test(api_deadline test_api_deadline.py TIMEOUT 45
    ENV "UNITREE_API_DEADLINE=$<TARGET_FILE:api_deadline_test>"
    "UNITREE_B2_API_DEADLINE=$<TARGET_FILE:b2_deadline_test>")
  foreach(model go2 b2)
    add_executable(legacy_${model}_sport_test legacy_sport_client_test.cpp)
    target_compile_features(legacy_${model}_sport_test PRIVATE cxx_std_17)
    target_include_directories(legacy_${model}_sport_test PRIVATE
      "../../example/src/include" "../../example/src/include/common")
    ament_target_dependencies(legacy_${model}_sport_test rclcpp unitree_api)
  endforeach()
  target_compile_definitions(legacy_b2_sport_test PRIVATE UNITREE_B2_SPORT)
  ament_add_pytest_test(legacy_sport test_legacy_sport_client.py TIMEOUT 40
    ENV "UNITREE_LEGACY_GO2_TEST=$<TARGET_FILE:legacy_go2_sport_test>"
    "UNITREE_LEGACY_B2_TEST=$<TARGET_FILE:legacy_b2_sport_test>")
  foreach(variant generic b2)
    add_executable(response_shape_${variant}_test response_shape_test.cpp)
    target_compile_features(response_shape_${variant}_test PRIVATE cxx_std_17)
    target_include_directories(response_shape_${variant}_test PRIVATE
      "../../example/src/include" "../../example/src/include/common"
      "../../example/src/include/nlohmann")
    ament_target_dependencies(response_shape_${variant}_test rclcpp unitree_api)
  endforeach()
  target_compile_definitions(response_shape_b2_test PRIVATE UNITREE_SHAPE_B2)
  ament_add_pytest_test(response_shape test_response_shape.py TIMEOUT 120
    ENV "UNITREE_SHAPE_GENERIC=$<TARGET_FILE:response_shape_generic_test>"
    "UNITREE_SHAPE_B2=$<TARGET_FILE:response_shape_b2_test>")
  foreach(variant single_flight_test b2_single_flight_test)
    add_executable(${variant} api_single_flight_test.cpp)
    target_compile_features(${variant} PRIVATE cxx_std_17)
    target_include_directories(${variant} PRIVATE
      "../../example/src/include" "../../example/src/include/common")
    ament_target_dependencies(${variant} rclcpp unitree_api)
  endforeach()
  target_compile_definitions(b2_single_flight_test PRIVATE UNITREE_B2_CLIENT)
  ament_add_pytest_test(api_single_flight test_api_single_flight.py TIMEOUT 240
    ENV "UNITREE_SINGLE_FLIGHT_TEST=$<TARGET_FILE:single_flight_test>"
    "UNITREE_B2_SINGLE_FLIGHT_TEST=$<TARGET_FILE:b2_single_flight_test>")
  foreach(variant request_qos_test b2_request_qos_test)
    add_executable(${variant} request_qos_test.cpp)
    target_compile_features(${variant} PRIVATE cxx_std_17)
    target_include_directories(${variant} PRIVATE
      "../../example/src/include" "../../example/src/include/common")
    ament_target_dependencies(${variant} rclcpp unitree_api)
  endforeach()
  target_compile_definitions(b2_request_qos_test PRIVATE UNITREE_B2_CLIENT)
  foreach(model go2 b2)
    add_executable(legacy_${model}_request_qos_test request_qos_test.cpp)
    target_compile_features(legacy_${model}_request_qos_test PRIVATE cxx_std_17)
    target_include_directories(legacy_${model}_request_qos_test PRIVATE
      "../../example/src/include" "../../example/src/include/common")
    ament_target_dependencies(legacy_${model}_request_qos_test rclcpp unitree_api)
    target_compile_definitions(legacy_${model}_request_qos_test PRIVATE UNITREE_LEGACY_SPORT)
  endforeach()
  target_compile_definitions(legacy_b2_request_qos_test PRIVATE UNITREE_B2_SPORT)
  add_executable(robot_state_request_qos_test request_qos_test.cpp)
  target_compile_features(robot_state_request_qos_test PRIVATE cxx_std_17)
  target_include_directories(robot_state_request_qos_test PRIVATE
    "../../example/src/include" "../../example/src/include/common")
  ament_target_dependencies(robot_state_request_qos_test rclcpp unitree_api)
  target_compile_definitions(robot_state_request_qos_test PRIVATE UNITREE_ROBOT_STATE)
  ament_add_pytest_test(request_qos test_request_qos.py TIMEOUT 60
    ENV "UNITREE_REQUEST_QOS_TEST=$<TARGET_FILE:request_qos_test>"
    "UNITREE_B2_REQUEST_QOS_TEST=$<TARGET_FILE:b2_request_qos_test>"
    "UNITREE_LEGACY_GO2_REQUEST_QOS_TEST=$<TARGET_FILE:legacy_go2_request_qos_test>"
    "UNITREE_LEGACY_B2_REQUEST_QOS_TEST=$<TARGET_FILE:legacy_b2_request_qos_test>"
    "UNITREE_ROBOT_STATE_REQUEST_QOS_TEST=$<TARGET_FILE:robot_state_request_qos_test>")
  ament_add_pytest_test(replay_tool test_replay_tool.py TIMEOUT 60)
  ament_add_pytest_test(bag_tool test_bag_tool.py TIMEOUT 60)
  ament_add_pytest_test(independent_bags test_compare_bags.py TIMEOUT 60)
  ament_add_pytest_test(interfaces test_interfaces.py TIMEOUT 120)
  ament_add_pytest_test(native_dds test_native_dds.py TIMEOUT 120
    ENV "UNITREE_DDS_PEER=$<TARGET_FILE:dds_peer>")
  ament_add_pytest_test(examples test_examples.py TIMEOUT 120)
  ament_add_pytest_test(setup_scripts test_setup.py TIMEOUT 30)
  ament_add_pytest_test(telemetry_preflight test_telemetry_preflight.py TIMEOUT 240
    ENV "UNITREE_TELEMETRY_ENDPOINT_PEER=$<TARGET_FILE:telemetry_endpoint_peer>")

  add_executable(api_burst_test api_burst_test.cpp)
  target_compile_features(api_burst_test PRIVATE cxx_std_17)
  target_include_directories(api_burst_test PRIVATE
    "../../example/src/include" "../../example/src/include/common")
  ament_target_dependencies(api_burst_test rclcpp unitree_api)
  ament_add_pytest_test(api_burst test_api_burst.py TIMEOUT 30
    ENV "UNITREE_API_BURST_TEST=$<TARGET_FILE:api_burst_test>")
endif()
ament_package()
