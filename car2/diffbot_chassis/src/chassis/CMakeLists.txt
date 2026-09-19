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
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# Shared CANopen core for ISWV and Yiyou; all sources belong to this package.
add_library(iswv_canopen STATIC
  src/motor/canopen/fake_transport.cpp
  src/motor/canopen/master.cpp
  src/motor/canopen/pdo.cpp
  src/motor/canopen/drive.cpp
  src/motor/canopen/axis.cpp
  src/motor/canopen/faults.cpp
  src/motor/canopen/module.cpp
  src/motor/canopen/steering_layout.cpp)
add_library(iswv::canopen ALIAS iswv_canopen)
set_target_properties(iswv_canopen PROPERTIES EXPORT_NAME canopen CXX_EXTENSIONS OFF)
target_compile_features(iswv_canopen PUBLIC cxx_std_17)
target_include_directories(iswv_canopen PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(iswv_canopen PUBLIC Threads::Threads)
target_compile_options(iswv_canopen PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wconversion -Wshadow>)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # Public .h files contain C++ interfaces.
  set(ament_cmake_uncrustify_ADDITIONAL_ARGS LANGUAGE C++)
  # Keep the supplied vendor ABI header unchanged; lint the maintained source files.
  set(AMENT_LINT_AUTO_FILE_EXCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/bxi_pci/include/bxi_pci_drv.h)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(chassis_bxi_transport STATIC src/motor/bxi_pci_transport.cpp)
target_include_directories(chassis_bxi_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/bxi_pci/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(chassis_bxi_transport PUBLIC
  iswv::canopen
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/vendor/bxi_pci/lib/libbxi_pci_drv.a>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/lib/libbxi_pci_drv.a>)

add_library(chassis_iswv_motor STATIC src/motor/iswv_motor.cpp)
target_include_directories(chassis_iswv_motor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(chassis_iswv_motor PUBLIC iswv::canopen yaml-cpp)

add_library(chassis_yiyou_motor STATIC src/motor/yiyou_motor.cpp)
target_include_directories(chassis_yiyou_motor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(chassis_yiyou_motor PUBLIC iswv::canopen)

add_library(chassis_bxi_motor STATIC src/motor/bxi_motor.cpp)
target_include_directories(chassis_bxi_motor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(chassis_bxi_motor PUBLIC cxx_std_17)
target_link_libraries(chassis_bxi_motor PUBLIC iswv::canopen)

add_library(chassis_arm_control STATIC src/control/arm_control.cpp)
target_link_libraries(chassis_arm_control PUBLIC chassis_bxi_motor chassis_bxi_transport)

add_library(chassis_controller STATIC src/control/chassis_control.cpp)
target_include_directories(chassis_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(chassis_controller PUBLIC cxx_std_17)
target_link_libraries(chassis_controller PUBLIC
  chassis_iswv_motor chassis_bxi_transport)
add_executable(chassis_main examples/chassis_main.cpp)
target_link_libraries(chassis_main PRIVATE chassis_controller chassis_arm_control)

add_executable(motor_power_hold examples/motor_power_hold.cpp)
target_link_libraries(motor_power_hold PRIVATE chassis_bxi_transport)

add_library(chassis_remote_control STATIC src/control/remote_control.cpp)
target_include_directories(chassis_remote_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(chassis_remote_control PUBLIC rclcpp rcl_interfaces geometry_msgs)

# Shared main: direct hardware startup or remote-button startup in one serial executor.
foreach(application chassis key_control robot_control)
  add_executable(${application} src/control/main.cpp)
  target_link_libraries(${application} PRIVATE
    chassis_controller chassis_arm_control chassis_remote_control)
endforeach()
target_compile_definitions(chassis PRIVATE CHASSIS_CONTROL_MODE="chassis")
target_compile_definitions(key_control PRIVATE CHASSIS_CONTROL_MODE="combined")
target_compile_definitions(robot_control PRIVATE CHASSIS_CONTROL_MODE="combined")

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(test_bxi_can3 tests/test_bxi_can3.cpp)
  target_link_libraries(test_bxi_can3 PRIVATE chassis_arm_control)
  add_test(NAME bxi_can3 COMMAND test_bxi_can3)
  set_tests_properties(bxi_can3 PROPERTIES TIMEOUT 10)
  add_test(NAME can3_terminal
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
    -s ${CMAKE_CURRENT_SOURCE_DIR}/tests -p test_can3_control.py)
  set_tests_properties(can3_terminal PROPERTIES TIMEOUT 10)
  add_executable(test_remote_start tests/test_remote_start.cpp)
  target_link_libraries(test_remote_start PRIVATE chassis_remote_control)
  add_test(NAME remote_start COMMAND test_remote_start)
  set_tests_properties(remote_start PROPERTIES TIMEOUT 30
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-test-logs")
  add_test(NAME remote_start_cli
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_remote_start_cli.py
    $<TARGET_FILE:key_control> $<TARGET_FILE:robot_control>)
  set_tests_properties(remote_start_cli PROPERTIES TIMEOUT 30)
  add_executable(test_can3_joystick tests/test_can3_joystick.cpp)
  target_include_directories(test_can3_joystick PRIVATE include)
  add_test(NAME can3_joystick COMMAND test_can3_joystick)
  add_executable(test_chassis_control tests/test_chassis_control.cpp)
  target_link_libraries(test_chassis_control PRIVATE chassis_controller)
  add_test(NAME chassis_control COMMAND test_chassis_control)
  set_tests_properties(chassis_control PROPERTIES TIMEOUT 10)
  foreach(protocol_test bxi_motor bxi_motor_commands)
    add_executable(test_${protocol_test} tests/test_${protocol_test}.cpp)
    target_link_libraries(test_${protocol_test} PRIVATE chassis_bxi_motor)
    add_test(NAME ${protocol_test} COMMAND test_${protocol_test})
  endforeach()
  add_executable(test_canopen
    tests/canopen/test_main.cpp tests/canopen/test_sdo.cpp
    tests/canopen/test_pdo.cpp tests/canopen/test_iswv.cpp)
  target_link_libraries(test_canopen PRIVATE iswv::canopen)
  add_test(NAME canopen COMMAND test_canopen)
  set_tests_properties(canopen PROPERTIES TIMEOUT 10)
  add_executable(bxi_motor_offline examples/bxi_motor_offline.cpp)
  target_link_libraries(bxi_motor_offline PRIVATE chassis_bxi_motor)
  add_test(NAME bxi_motor_offline COMMAND bxi_motor_offline)
  foreach(application chassis key_control robot_control)
    add_test(NAME ${application}_help COMMAND ${application} --help)
    set_tests_properties(${application}_help PROPERTIES
      PASS_REGULAR_EXPRESSION "Help does not open CAN devices")
  endforeach()
  add_test(NAME remote_can3_integration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_remote_can3_integration.py
    $<TARGET_FILE:key_control>)
  set_tests_properties(remote_can3_integration PROPERTIES TIMEOUT 60)
  add_executable(test_remote_composition tests/test_remote_composition.cpp)
  target_link_libraries(test_remote_composition PRIVATE chassis_remote_control)
  add_test(NAME remote_composition COMMAND test_remote_composition)
  set_tests_properties(remote_composition PROPERTIES TIMEOUT 15
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-test-logs")
  # CLI checks use the application targets and do not require separate test sources.
  add_test(NAME chassis_main_help COMMAND chassis_main --help)
  add_test(NAME motor_power_hold_help COMMAND motor_power_hold --help)
  set_tests_properties(motor_power_hold_help PROPERTIES
    PASS_REGULAR_EXPRESSION "does not enable or command any motor")
endif()

install(TARGETS
  chassis
  chassis_main
  motor_power_hold
  key_control
  robot_control
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/can3_control.py DESTINATION lib/${PROJECT_NAME})

install(TARGETS chassis_controller chassis_iswv_motor chassis_yiyou_motor
  chassis_bxi_motor chassis_bxi_transport chassis_arm_control chassis_remote_control
  EXPORT chassisControlTargets
  ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES vendor/bxi_pci/include/bxi_pci_drv.h
  DESTINATION include)
install(FILES vendor/bxi_pci/lib/libbxi_pci_drv.a
  DESTINATION lib)

# Preserve the public dependency target for programs using the installed motor APIs.
include(CMakePackageConfigHelpers)
install(TARGETS iswv_canopen EXPORT iswv_canopenTargets ARCHIVE DESTINATION lib)
install(EXPORT iswv_canopenTargets NAMESPACE iswv:: DESTINATION lib/cmake/iswv_canopen)
configure_package_config_file(cmake/iswv_canopenConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/iswv_canopenConfig.cmake
  INSTALL_DESTINATION lib/cmake/iswv_canopen)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/iswv_canopenConfigVersion.cmake
  VERSION 0.1.0 COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/iswv_canopenConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/iswv_canopenConfigVersion.cmake
  DESTINATION lib/cmake/iswv_canopen)
ament_export_include_directories(include)
ament_export_dependencies(iswv_canopen yaml-cpp rclcpp rcl_interfaces geometry_msgs)
ament_export_targets(chassisControlTargets HAS_LIBRARY_TARGET)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
