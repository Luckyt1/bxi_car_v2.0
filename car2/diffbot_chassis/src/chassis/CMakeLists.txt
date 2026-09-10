cmake_minimum_required(VERSION 3.8)
project(chassis)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

set(ISWV_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../ISWV" CACHE PATH
  "Path to the iSWV CANopen driver source tree")
if(NOT EXISTS "${ISWV_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR "iSWV driver not found: set ISWV_SOURCE_DIR")
endif()
set(ISWV_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ISWV_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(ISWV_BUILD_ZQWL_TRANSPORT OFF CACHE BOOL "" FORCE)
set(ISWV_BUILD_ROS2_TRANSPORT OFF CACHE BOOL "" FORCE)
add_subdirectory("${ISWV_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/iswv_canopen")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(chassis_bxi_transport STATIC src/bxi_pci_transport.cpp)
target_include_directories(chassis_bxi_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../bxi_pci_drv/lib>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(chassis_bxi_transport PUBLIC
  iswv::canopen
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../bxi_pci_drv/lib/libbxi_pci_drv.a>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/lib/libbxi_pci_drv.a>)

add_executable(chassis src/chassis.cpp)
add_executable(steering_calibration src/steering_calibration.cpp)

add_library(chassis_iswv_drive STATIC src/iswv_drive.cpp)
target_include_directories(chassis_iswv_drive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(chassis_iswv_drive PUBLIC iswv::canopen)
add_library(chassis_calibration STATIC src/steering_calibration_core.cpp src/steering_direction_core.cpp)
target_include_directories(chassis_calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(chassis_calibration PUBLIC iswv::canopen yaml-cpp)
add_library(chassis_controller STATIC src/chassis_controller.cpp)
target_include_directories(chassis_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(chassis_controller PUBLIC cxx_std_17)
target_link_libraries(chassis_controller PUBLIC
  chassis_iswv_drive chassis_calibration chassis_bxi_transport)
add_executable(chassis_main examples/chassis_main.cpp)
target_link_libraries(chassis_main PRIVATE chassis_controller)

add_library(chassis_eyou_motor STATIC src/eyou_motor.cpp)
target_include_directories(chassis_eyou_motor PUBLIC include)
target_link_libraries(chassis_eyou_motor PUBLIC iswv::canopen)
add_executable(eyou_motor_test src/eyou_motor_test.cpp)
target_link_libraries(eyou_motor_test PRIVATE chassis_eyou_motor chassis_bxi_transport)
add_executable(eyou_motor_info src/eyou_motor_info.cpp)
target_link_libraries(eyou_motor_info PRIVATE chassis_eyou_motor chassis_bxi_transport)
add_executable(motor_power_hold src/motor_power_hold.cpp)
target_link_libraries(motor_power_hold PRIVATE chassis_bxi_transport)
add_executable(drive_wheel_test src/drive_wheel_test.cpp)
add_executable(steering_direction_test src/steering_direction_test.cpp)
target_link_libraries(steering_direction_test PRIVATE chassis_bxi_transport yaml-cpp)
target_link_libraries(drive_wheel_test PRIVATE
  chassis_iswv_drive chassis_bxi_transport yaml-cpp)

add_executable(key_control src/control.cpp)
ament_target_dependencies(key_control PUBLIC rclcpp geometry_msgs)

target_link_libraries(chassis PRIVATE chassis_controller)
target_link_libraries(steering_calibration PRIVATE chassis_bxi_transport chassis_calibration)

ament_target_dependencies(chassis PUBLIC rclcpp geometry_msgs)
ament_target_dependencies(steering_calibration PUBLIC rclcpp yaml_cpp_vendor)

if(BUILD_TESTING)
  add_executable(test_positive_steering_direction tests/test_positive_steering_direction.cpp)
  target_link_libraries(test_positive_steering_direction PRIVATE chassis_calibration)
  add_test(NAME test_positive_steering_direction COMMAND test_positive_steering_direction)
  add_executable(test_steering_direction_cli tests/test_steering_direction_cli.cpp)
  target_link_libraries(test_steering_direction_cli PRIVATE chassis_bxi_transport yaml-cpp)
  add_test(NAME test_steering_direction_cli COMMAND test_steering_direction_cli
    $<TARGET_FILE:steering_direction_test> ${CMAKE_CURRENT_SOURCE_DIR}/config/steering.yaml)
  add_test(NAME chassis_main_help COMMAND chassis_main --help)
  foreach(test_name chassis_config iswv_drive steering_calibration chassis_controller)
    add_executable(test_${test_name} tests/test_${test_name}.cpp)
    target_link_libraries(test_${test_name} PRIVATE chassis_controller)
    add_test(NAME test_${test_name} COMMAND test_${test_name})
    set_tests_properties(test_${test_name} PROPERTIES TIMEOUT 60)
  endforeach()
  add_executable(test_bxi_pci_transport tests/test_bxi_pci_transport.cpp)
  target_link_libraries(test_bxi_pci_transport PRIVATE chassis_bxi_transport)
  add_test(NAME test_bxi_pci_transport COMMAND test_bxi_pci_transport)
  add_executable(test_steering_config tests/test_steering_config.cpp)
  target_include_directories(test_steering_config PRIVATE include)
  target_link_libraries(test_steering_config PRIVATE iswv::canopen yaml-cpp)
  add_test(NAME test_steering_config COMMAND test_steering_config
    ${CMAKE_CURRENT_SOURCE_DIR}/config/steering.yaml)
  add_executable(test_swerve_kinematics tests/test_swerve_kinematics.cpp)
  target_include_directories(test_swerve_kinematics PRIVATE include)
  add_test(NAME test_swerve_kinematics COMMAND test_swerve_kinematics)
  add_executable(test_eyou_motor tests/test_eyou_motor.cpp)
  target_link_libraries(test_eyou_motor PRIVATE chassis_eyou_motor)
  add_test(NAME test_eyou_motor COMMAND test_eyou_motor)
  add_executable(test_torque_limit_detector tests/test_torque_limit_detector.cpp)
  target_include_directories(test_torque_limit_detector PRIVATE include)
  add_test(NAME test_torque_limit_detector COMMAND test_torque_limit_detector)
  add_test(NAME eyou_motor_info_help COMMAND eyou_motor_info --help)
  set_tests_properties(eyou_motor_info_help PROPERTIES
    PASS_REGULAR_EXPRESSION "Only SDO reads")
  add_test(NAME eyou_motor_info_invalid_node COMMAND eyou_motor_info 128)
  set_tests_properties(eyou_motor_info_invalid_node PROPERTIES WILL_FAIL TRUE)
  add_test(NAME eyou_motor_info_invalid_node_with_bus COMMAND eyou_motor_info 128 1)
  set_tests_properties(eyou_motor_info_invalid_node_with_bus PROPERTIES WILL_FAIL TRUE)
  add_test(NAME eyou_motor_test_help COMMAND eyou_motor_test --help)
  set_tests_properties(eyou_motor_test_help PROPERTIES
    PASS_REGULAR_EXPRESSION "--continuous")
  add_test(NAME motor_power_hold_help COMMAND motor_power_hold --help)
  set_tests_properties(motor_power_hold_help PROPERTIES
    PASS_REGULAR_EXPRESSION "does not enable or command any motor")
  add_executable(test_motor_power_hold_cli tests/test_motor_power_hold_cli.cpp)
  target_include_directories(test_motor_power_hold_cli PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../bxi_pci_drv/lib)
  target_link_libraries(test_motor_power_hold_cli PRIVATE iswv::canopen)
  add_test(NAME test_motor_power_hold_cli COMMAND test_motor_power_hold_cli)
  add_test(NAME drive_wheel_test_help COMMAND drive_wheel_test --help)
  set_tests_properties(drive_wheel_test_help PROPERTIES
    PASS_REGULAR_EXPRESSION "Only the selected ISWV travel motor is enabled")
  add_test(NAME drive_wheel_test_invalid_wheel
    COMMAND drive_wheel_test ${CMAKE_CURRENT_SOURCE_DIR}/config/steering.yaml 5 0.5)
  set_tests_properties(drive_wheel_test_invalid_wheel PROPERTIES WILL_FAIL TRUE)
  add_test(NAME drive_wheel_test_zero_rpm
    COMMAND drive_wheel_test ${CMAKE_CURRENT_SOURCE_DIR}/config/steering.yaml 1 0)
  set_tests_properties(drive_wheel_test_zero_rpm PROPERTIES WILL_FAIL TRUE)
  add_executable(test_eyou_motor_test_cli tests/test_eyou_motor_test_cli.cpp)
  target_include_directories(test_eyou_motor_test_cli PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../bxi_pci_drv/lib)
  target_link_libraries(test_eyou_motor_test_cli PRIVATE chassis_eyou_motor)
  add_test(NAME test_eyou_motor_test_cli COMMAND test_eyou_motor_test_cli)
  set_tests_properties(test_eyou_motor_test_cli PROPERTIES TIMEOUT 35)
endif()

install(TARGETS
  chassis
  chassis_main
  steering_calibration
  eyou_motor_test
  eyou_motor_info
  motor_power_hold
  drive_wheel_test
  steering_direction_test
  key_control
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS chassis_controller chassis_iswv_drive chassis_calibration chassis_bxi_transport
  EXPORT chassisControlTargets
  ARCHIVE DESTINATION lib)
install(DIRECTORY include/chassis DESTINATION include)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../bxi_pci_drv/lib/bxi_pci_drv.h
  DESTINATION include)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../bxi_pci_drv/lib/libbxi_pci_drv.a
  DESTINATION lib)
ament_export_include_directories(include)
ament_export_dependencies(iswv_canopen yaml-cpp)
ament_export_targets(chassisControlTargets HAS_LIBRARY_TARGET)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
