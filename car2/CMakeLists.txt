cmake_minimum_required(VERSION 3.16)
project(chassis LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Keep clangd's include paths and compile flags in sync with normal colcon builds.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Application, modules, configuration and package templates live under src/.
set(CHASSIS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

# Protocol internals stay separate; the two public entry files are communication/motor.
set(ISWV_PROTOCOL "${CHASSIS_SOURCE_DIR}/thirdparty/motor/iswv/protocol")
add_library(iswv_canopen STATIC
  "${CHASSIS_SOURCE_DIR}/thirdparty/motor/iswv/communication.cpp"
  ${ISWV_PROTOCOL}/src/fake_transport.cpp
  ${ISWV_PROTOCOL}/src/pdo.cpp
  ${ISWV_PROTOCOL}/src/drive.cpp
  ${ISWV_PROTOCOL}/src/axis.cpp
  ${ISWV_PROTOCOL}/src/faults.cpp
  ${ISWV_PROTOCOL}/src/module.cpp
  ${ISWV_PROTOCOL}/src/steering_layout.cpp)
add_library(iswv::canopen ALIAS iswv_canopen)
set_target_properties(iswv_canopen PROPERTIES EXPORT_NAME canopen CXX_EXTENSIONS OFF)
target_compile_features(iswv_canopen PUBLIC cxx_std_17)
target_include_directories(iswv_canopen PUBLIC
  $<BUILD_INTERFACE:${CHASSIS_SOURCE_DIR}/thirdparty>
  $<BUILD_INTERFACE:${ISWV_PROTOCOL}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(iswv_canopen PUBLIC Threads::Threads)
target_compile_options(iswv_canopen PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wconversion -Wshadow>)

add_library(chassis_bxi_transport STATIC "${CHASSIS_SOURCE_DIR}/thirdparty/bxi/board.cpp")
target_include_directories(chassis_bxi_transport PUBLIC
  $<BUILD_INTERFACE:${CHASSIS_SOURCE_DIR}/thirdparty>
  $<BUILD_INTERFACE:${CHASSIS_SOURCE_DIR}/thirdparty/bxi/vendor/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(chassis_bxi_transport PUBLIC Threads::Threads
  $<BUILD_INTERFACE:${CHASSIS_SOURCE_DIR}/thirdparty/bxi/vendor/lib/libbxi_pci_drv.a>
  $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/lib/libbxi_pci_drv.a>)

add_library(chassis_bxi_motor STATIC
  "${CHASSIS_SOURCE_DIR}/thirdparty/motor/bxi/communication.cpp"
  "${CHASSIS_SOURCE_DIR}/thirdparty/motor/bxi/motor.cpp")
target_include_directories(chassis_bxi_motor PUBLIC
  $<BUILD_INTERFACE:${CHASSIS_SOURCE_DIR}/thirdparty> $<INSTALL_INTERFACE:include>)
add_library(chassis_iswv_bxi_transport STATIC
  "${CHASSIS_SOURCE_DIR}/thirdparty/motor/iswv/bxi_transport.cpp")
target_link_libraries(chassis_iswv_bxi_transport PUBLIC iswv::canopen)
add_library(chassis_iswv_motor STATIC "${CHASSIS_SOURCE_DIR}/thirdparty/motor/iswv/motor.cpp")
target_link_libraries(chassis_iswv_motor PUBLIC iswv::canopen)

set(SOEM "${CHASSIS_SOURCE_DIR}/thirdparty/motor/yiyou/vendor/soem")
file(GLOB SOEM_SOURCES CONFIGURE_DEPENDS ${SOEM}/soem/*.c)
add_library(chassis_soem STATIC ${SOEM_SOURCES}
  ${SOEM}/osal/linux/osal.c ${SOEM}/oshw/linux/oshw.c ${SOEM}/oshw/linux/nicdrv.c)
target_include_directories(chassis_soem PRIVATE
  ${SOEM}/soem ${SOEM}/osal ${SOEM}/osal/linux ${SOEM}/oshw/linux)
target_link_libraries(chassis_soem PUBLIC Threads::Threads rt)
add_library(chassis_ethercat STATIC "${CHASSIS_SOURCE_DIR}/thirdparty/motor/yiyou/communication.cpp")
target_include_directories(chassis_ethercat PRIVATE
  ${SOEM}/soem ${SOEM}/osal ${SOEM}/osal/linux ${SOEM}/oshw/linux)
target_link_libraries(chassis_ethercat PUBLIC iswv::canopen chassis_soem)
add_library(chassis_yiyou_motor STATIC "${CHASSIS_SOURCE_DIR}/thirdparty/motor/yiyou/motor.cpp")
target_link_libraries(chassis_yiyou_motor PUBLIC chassis_ethercat)

add_library(chassis_arm_kinematics STATIC "${CHASSIS_SOURCE_DIR}/thirdparty/arm/kinematics.cpp")
target_include_directories(chassis_arm_kinematics PUBLIC
  $<BUILD_INTERFACE:${CHASSIS_SOURCE_DIR}/thirdparty> $<INSTALL_INTERFACE:include>)
add_library(chassis_arm_control STATIC "${CHASSIS_SOURCE_DIR}/thirdparty/arm/control.cpp")
target_link_libraries(chassis_arm_control PUBLIC
  chassis_arm_kinematics chassis_yiyou_motor chassis_bxi_transport yaml-cpp)
add_library(chassis_controller STATIC "${CHASSIS_SOURCE_DIR}/thirdparty/chassis/control.cpp")
target_link_libraries(chassis_controller PUBLIC
  chassis_iswv_motor chassis_iswv_bxi_transport chassis_bxi_transport yaml-cpp)

# ROS belongs to the application layer; all hardware/domain modules also work without ROS.
add_library(chassis_remote_control STATIC
  "${CHASSIS_SOURCE_DIR}/example/remote_control.cpp"
  "${CHASSIS_SOURCE_DIR}/example/joystick_link_monitor.cpp")
target_include_directories(chassis_remote_control PUBLIC
  $<BUILD_INTERFACE:${CHASSIS_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CHASSIS_SOURCE_DIR}/thirdparty> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(chassis_remote_control PUBLIC rclcpp rcl_interfaces geometry_msgs std_msgs)
target_link_libraries(chassis_remote_control PUBLIC yaml-cpp Threads::Threads)
# Keep existing ROS executable names; there is only one maintained main.cpp.
foreach(application chassis key_control robot_control chassis_only chassis_only_remote)
  add_executable(${application} "${CHASSIS_SOURCE_DIR}/example/main.cpp")
  target_compile_definitions(${application} PRIVATE
    CHASSIS_SOURCE_CONFIG_DIR="${CHASSIS_SOURCE_DIR}/config")
  target_link_libraries(${application} PRIVATE
    chassis_controller chassis_arm_control chassis_remote_control)
endforeach()
target_compile_definitions(chassis PRIVATE CHASSIS_CONTROL_MODE="chassis")
target_compile_definitions(key_control PRIVATE CHASSIS_CONTROL_MODE="combined")
target_compile_definitions(robot_control PRIVATE CHASSIS_CONTROL_MODE="combined")
target_compile_definitions(chassis_only PRIVATE CHASSIS_CONTROL_MODE="chassis_only")
target_compile_definitions(chassis_only_remote PRIVATE CHASSIS_CONTROL_MODE="chassis_only_remote")

# Diagnostic backends are reached through test/debug.py.
foreach(tool arm_yiyou_test arm_yiyou_set_id arm_yiyou_control)
  add_executable(${tool} test/tools/${tool}.cpp)
  target_link_libraries(${tool} PRIVATE chassis_yiyou_motor chassis_bxi_transport)
endforeach()
add_executable(chassis_main test/tools/chassis_main.cpp)
target_link_libraries(chassis_main PRIVATE chassis_controller chassis_arm_control)
add_executable(motor_power_hold test/tools/motor_power_hold.cpp)
target_link_libraries(motor_power_hold PRIVATE chassis_bxi_transport)
add_executable(bxi_motor_offline test/tools/bxi_motor_offline.cpp)
target_link_libraries(bxi_motor_offline PRIVATE chassis_bxi_motor)
add_executable(motor_console test/tools/motor_console.cpp)
target_link_libraries(motor_console PRIVATE
  chassis_bxi_motor chassis_iswv_motor chassis_iswv_bxi_transport chassis_bxi_transport)
add_executable(arm_solver test/tools/arm_solver.cpp)
target_link_libraries(arm_solver PRIVATE chassis_arm_control)

# Give editable examples accurate compile commands without running hardware.
foreach(brand iswv yiyou)
  add_library(${brand}_motor_example OBJECT EXCLUDE_FROM_ALL
    "${CHASSIS_SOURCE_DIR}/thirdparty/motor/${brand}/example.cpp")
  target_link_libraries(${brand}_motor_example PRIVATE chassis_${brand}_motor)
endforeach()
add_executable(bxi_motor_example EXCLUDE_FROM_ALL
  "${CHASSIS_SOURCE_DIR}/thirdparty/motor/bxi/example.cpp")
target_link_libraries(bxi_motor_example PRIVATE chassis_bxi_motor chassis_bxi_transport)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(test_steering_config test/unit/test_steering_config.cpp)
  target_link_libraries(test_steering_config PRIVATE iswv::canopen yaml-cpp)
  add_test(NAME steering_config COMMAND test_steering_config
    ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/fixtures/steering.yaml
    ${CHASSIS_SOURCE_DIR}/config/steering.yaml)
  add_executable(test_regression test/unit/regression.cpp)
  target_link_libraries(test_regression PRIVATE
    chassis_bxi_motor chassis_arm_kinematics chassis_ethercat)
  add_test(NAME regression COMMAND test_regression)
  add_executable(test_motor_layers test/unit/test_motor_layers.cpp)
  target_link_libraries(test_motor_layers PRIVATE
    chassis_bxi_motor chassis_iswv_motor chassis_yiyou_motor)
  add_test(NAME motor_layers COMMAND test_motor_layers)
  set_tests_properties(motor_layers PROPERTIES TIMEOUT 30)
  add_executable(test_bxi_transport_adapter test/unit/test_bxi_transport_adapter.cpp)
  target_link_libraries(test_bxi_transport_adapter PRIVATE chassis_iswv_bxi_transport)
  add_test(NAME bxi_transport_adapter COMMAND test_bxi_transport_adapter)
  set_tests_properties(bxi_transport_adapter PROPERTIES TIMEOUT 10)
  add_test(NAME debug_menu COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_debug.py)
  set_tests_properties(debug_menu PROPERTIES TIMEOUT 30
    ENVIRONMENT "CHASSIS_TEST_BIN_DIR=${CMAKE_CURRENT_BINARY_DIR};PYTHONDONTWRITEBYTECODE=1")
  add_test(NAME startup_scripts COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_start_scripts.py)
  set_tests_properties(startup_scripts PROPERTIES TIMEOUT 30
    ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")
  add_test(NAME ros_contract COMMAND ${Python3_EXECUTABLE} -B
    ${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_ros_contract.py
    --bin-dir ${CMAKE_CURRENT_BINARY_DIR} --config ${CHASSIS_SOURCE_DIR}/config/steering.yaml)
  set_tests_properties(ros_contract PROPERTIES TIMEOUT 45
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-test-logs")
  add_executable(test_remote_disconnect test/unit/test_remote_disconnect.cpp)
  target_link_libraries(test_remote_disconnect PRIVATE chassis_remote_control)
  add_test(NAME remote_disconnect COMMAND test_remote_disconnect)
  set_tests_properties(remote_disconnect PROPERTIES TIMEOUT 25
    ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=197;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-test-logs")
  add_executable(test_velocity_command_watchdog test/unit/test_velocity_command_watchdog.cpp)
  target_include_directories(test_velocity_command_watchdog PRIVATE ${CHASSIS_SOURCE_DIR})
  add_test(NAME velocity_command_watchdog COMMAND test_velocity_command_watchdog)
  add_executable(test_joystick_link_monitor test/unit/test_joystick_link_monitor.cpp)
  target_link_libraries(test_joystick_link_monitor PRIVATE chassis_remote_control)
  add_test(NAME joystick_link_monitor COMMAND test_joystick_link_monitor)
  set_tests_properties(joystick_link_monitor PROPERTIES TIMEOUT 15)
  foreach(application chassis key_control robot_control chassis_only chassis_only_remote
      arm_yiyou_control arm_yiyou_test arm_yiyou_set_id chassis_main motor_power_hold motor_console arm_solver)
    add_test(NAME ${application}_help COMMAND ${application} --help)
    set_tests_properties(${application}_help PROPERTIES TIMEOUT 5)
  endforeach()
  add_test(NAME bxi_motor_offline COMMAND bxi_motor_offline)
endif()

install(TARGETS chassis key_control robot_control chassis_only chassis_only_remote
  arm_yiyou_test arm_yiyou_set_id arm_yiyou_control chassis_main motor_power_hold motor_console arm_solver
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS test/debug.py test/tools/yiyou_control.py test/tools/can3_control.py
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS chassis_bxi_transport chassis_bxi_motor chassis_iswv_motor chassis_iswv_bxi_transport
  chassis_soem chassis_ethercat chassis_yiyou_motor chassis_arm_kinematics
  chassis_arm_control chassis_controller chassis_remote_control
  EXPORT chassisControlTargets ARCHIVE DESTINATION lib)
install(DIRECTORY "${CHASSIS_SOURCE_DIR}/thirdparty/" DESTINATION include FILES_MATCHING
  PATTERN "*.h" PATTERN "*.hpp" PATTERN "vendor" EXCLUDE PATTERN "protocol" EXCLUDE)
install(DIRECTORY ${ISWV_PROTOCOL}/include/ DESTINATION include)
install(DIRECTORY "${CHASSIS_SOURCE_DIR}/example/" DESTINATION include/example FILES_MATCHING
  PATTERN "*.h" PATTERN "*.hpp")
install(FILES "${CHASSIS_SOURCE_DIR}/thirdparty/bxi/vendor/include/bxi_pci_drv.h" DESTINATION include)
install(FILES "${CHASSIS_SOURCE_DIR}/thirdparty/bxi/vendor/lib/libbxi_pci_drv.a" DESTINATION lib)

include(CMakePackageConfigHelpers)
install(TARGETS iswv_canopen EXPORT iswv_canopenTargets ARCHIVE DESTINATION lib)
install(EXPORT iswv_canopenTargets NAMESPACE iswv:: DESTINATION lib/cmake/iswv_canopen)
configure_package_config_file("${CHASSIS_SOURCE_DIR}/cmake/iswv_canopenConfig.cmake.in"
  ${CMAKE_CURRENT_BINARY_DIR}/iswv_canopenConfig.cmake INSTALL_DESTINATION lib/cmake/iswv_canopen)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/iswv_canopenConfigVersion.cmake
  VERSION 0.1.0 COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/iswv_canopenConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/iswv_canopenConfigVersion.cmake DESTINATION lib/cmake/iswv_canopen)
ament_export_include_directories(include)
ament_export_dependencies(iswv_canopen yaml-cpp rclcpp rcl_interfaces geometry_msgs std_msgs)
ament_export_targets(chassisControlTargets HAS_LIBRARY_TARGET)
install(DIRECTORY "${CHASSIS_SOURCE_DIR}/config/" DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY docs/ DESTINATION share/${PROJECT_NAME}/docs)
install(FILES LICENSE README.md DESTINATION share/${PROJECT_NAME})
install(FILES ${SOEM}/LICENSE ${SOEM}/ORIGIN.md ${SOEM}/LOCAL_PATCHES.md
  DESTINATION share/${PROJECT_NAME}/vendor/soem)
ament_package()
