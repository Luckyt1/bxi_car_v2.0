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

set(ISWV_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../ISWV" CACHE PATH
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

add_library(chassis_iswv_motor STATIC src/iswv_motor.cpp)
target_include_directories(chassis_iswv_motor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(chassis_iswv_motor PUBLIC iswv::canopen yaml-cpp)

add_library(chassis_eyou_motor STATIC src/eyou_motor.cpp)
target_include_directories(chassis_eyou_motor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(chassis_eyou_motor PUBLIC iswv::canopen)

add_library(chassis_bxi_motor STATIC src/bxi_motor.cpp)
target_include_directories(chassis_bxi_motor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(chassis_bxi_motor PUBLIC cxx_std_17)
target_link_libraries(chassis_bxi_motor PUBLIC iswv::canopen)

add_library(chassis_bxi_can3 STATIC src/bxi_can3.cpp)
target_link_libraries(chassis_bxi_can3 PUBLIC chassis_bxi_motor chassis_bxi_transport)

add_library(chassis_controller STATIC src/chassis_controller.cpp)
target_include_directories(chassis_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_compile_features(chassis_controller PUBLIC cxx_std_17)
target_link_libraries(chassis_controller PUBLIC
  chassis_iswv_motor chassis_bxi_transport)
add_executable(chassis_main examples/chassis_main.cpp)
target_link_libraries(chassis_main PRIVATE chassis_controller chassis_bxi_can3)

add_executable(motor_power_hold src/motor_power_hold.cpp)
target_link_libraries(motor_power_hold PRIVATE chassis_bxi_transport)

add_executable(key_control src/control.cpp)
target_include_directories(key_control PRIVATE include)
ament_target_dependencies(key_control PUBLIC rclcpp rcl_interfaces geometry_msgs)

target_link_libraries(chassis PRIVATE chassis_controller chassis_bxi_can3)

ament_target_dependencies(chassis PUBLIC rclcpp rcl_interfaces geometry_msgs)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(test_bxi_can3 tests/test_bxi_can3.cpp)
  target_link_libraries(test_bxi_can3 PRIVATE chassis_bxi_can3)
  add_test(NAME bxi_can3 COMMAND test_bxi_can3)
  set_tests_properties(bxi_can3 PROPERTIES TIMEOUT 10)
  add_test(NAME can3_terminal
    COMMAND ${Python3_EXECUTABLE} -m unittest discover
    -s ${CMAKE_CURRENT_SOURCE_DIR}/tests -p test_can3_control.py)
  set_tests_properties(can3_terminal PROPERTIES TIMEOUT 10)
  add_executable(test_can3_joystick tests/test_can3_joystick.cpp)
  target_include_directories(test_can3_joystick PRIVATE include)
  add_test(NAME can3_joystick COMMAND test_can3_joystick)
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
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/can3_control.py DESTINATION lib/${PROJECT_NAME})

install(TARGETS chassis_controller chassis_iswv_motor chassis_eyou_motor
  chassis_bxi_motor chassis_bxi_transport chassis_bxi_can3
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
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

ament_package()
