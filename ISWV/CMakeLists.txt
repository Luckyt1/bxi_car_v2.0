cmake_minimum_required(VERSION 3.16)

project(iswv_canopen VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if("${CMAKE_EXPORT_COMPILE_COMMANDS}" STREQUAL "")
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL
        "Generate compile_commands.json for clangd and other tooling" FORCE)
endif()

option(ISWV_BUILD_TESTS "Build unit tests" ON)
option(ISWV_BUILD_EXAMPLES "Build examples" ON)
option(ISWV_BUILD_ZQWL_TRANSPORT "Build the optional ZqwlCan transport" OFF)
option(ISWV_BUILD_ROS2_TRANSPORT "Build the optional ROS 2 topic transport" OFF)
set(ISWV_ZQWL_SOURCE_DIR "" CACHE PATH
    "Optional path to the zqwl_can_linux source tree when no installed package is available")

find_package(Threads REQUIRED)

add_library(iswv_canopen
    src/fake_transport.cpp
    src/canopen/master.cpp
    src/canopen/pdo.cpp
    src/cia402/drive.cpp
    src/iswv/axis.cpp
    src/iswv/faults.cpp
    src/iswv/module.cpp
    src/iswv/steering_layout.cpp)
add_library(iswv::canopen ALIAS iswv_canopen)
set_target_properties(iswv_canopen PROPERTIES EXPORT_NAME canopen)

target_compile_features(iswv_canopen PUBLIC cxx_std_17)
target_include_directories(iswv_canopen
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(iswv_canopen PUBLIC Threads::Threads)
target_compile_options(iswv_canopen PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wconversion -Wshadow>)

if(ISWV_BUILD_ZQWL_TRANSPORT)
    if(NOT TARGET zqwl_can::zqwl_can)
        find_package(zqwl_can CONFIG QUIET)
    endif()
    if(NOT TARGET zqwl_can::zqwl_can AND ISWV_ZQWL_SOURCE_DIR)
        set(ZQWL_CAN_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
        set(ZQWL_CAN_BUILD_TESTS OFF CACHE BOOL "" FORCE)
        set(ZQWL_CAN_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        add_subdirectory("${ISWV_ZQWL_SOURCE_DIR}"
                         "${CMAKE_CURRENT_BINARY_DIR}/zqwl_can" EXCLUDE_FROM_ALL)
    endif()
    if(NOT TARGET zqwl_can::zqwl_can)
        message(FATAL_ERROR
            "ZqwlCan was requested but not found. Install zqwl_can or set ISWV_ZQWL_SOURCE_DIR.")
    endif()
    add_library(iswv_transport_zqwl src/transports/zqwl_transport.cpp)
    add_library(iswv::transport_zqwl ALIAS iswv_transport_zqwl)
    set_target_properties(iswv_transport_zqwl PROPERTIES EXPORT_NAME transport_zqwl)
    target_compile_features(iswv_transport_zqwl PUBLIC cxx_std_17)
    target_include_directories(iswv_transport_zqwl
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
    target_link_libraries(iswv_transport_zqwl PUBLIC iswv_canopen zqwl_can::zqwl_can)
    target_compile_definitions(iswv_transport_zqwl PUBLIC ISWV_HAS_ZQWL_TRANSPORT=1)
    target_compile_options(iswv_transport_zqwl PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wconversion -Wshadow>)
endif()

if(ISWV_BUILD_ROS2_TRANSPORT)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(communication REQUIRED)

    add_library(iswv_transport_ros2 src/transports/ros2_transport.cpp)
    add_library(iswv::transport_ros2 ALIAS iswv_transport_ros2)
    set_target_properties(iswv_transport_ros2 PROPERTIES EXPORT_NAME transport_ros2)
    target_compile_features(iswv_transport_ros2 PUBLIC cxx_std_17)
    target_include_directories(iswv_transport_ros2
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
    target_link_libraries(iswv_transport_ros2 iswv_canopen)
    ament_target_dependencies(iswv_transport_ros2 rclcpp communication)
    target_compile_definitions(iswv_transport_ros2 PUBLIC ISWV_HAS_ROS2_TRANSPORT=1)
    target_compile_options(iswv_transport_ros2 PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wconversion -Wshadow>)
endif()

if(ISWV_BUILD_EXAMPLES)
    add_executable(iswv_basic_example examples/basic.cpp)
    target_link_libraries(iswv_basic_example PRIVATE iswv_canopen)
    if(TARGET iswv_transport_zqwl)
        add_executable(iswv_zqwl_example examples/zqwl.cpp)
        target_link_libraries(iswv_zqwl_example PRIVATE iswv_transport_zqwl)
        add_executable(iswv_zqwl_slow_rotate_example examples/zqwl_slow_rotate.cpp)
        target_link_libraries(iswv_zqwl_slow_rotate_example
            PRIVATE iswv_transport_zqwl)
    endif()
    if(TARGET iswv_transport_ros2)
        add_executable(iswv_ros2_example examples/ros2_node.cpp)
        target_link_libraries(iswv_ros2_example iswv_transport_ros2)
        ament_target_dependencies(iswv_ros2_example rclcpp communication)
        install(TARGETS iswv_ros2_example DESTINATION lib/${PROJECT_NAME})
    endif()
endif()

if(ISWV_BUILD_TESTS)
    enable_testing()
    add_executable(iswv_tests
        tests/test_main.cpp
        tests/test_sdo.cpp
        tests/test_pdo.cpp
        tests/test_iswv.cpp)
    target_link_libraries(iswv_tests PRIVATE iswv_canopen)
    target_compile_options(iswv_tests PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wconversion -Wshadow>)
    add_test(NAME iswv_tests COMMAND iswv_tests)
endif()

set(ISWV_INSTALL_TARGETS iswv_canopen)
if(TARGET iswv_transport_zqwl)
    list(APPEND ISWV_INSTALL_TARGETS iswv_transport_zqwl)
endif()
if(TARGET iswv_transport_ros2)
    list(APPEND ISWV_INSTALL_TARGETS iswv_transport_ros2)
endif()

install(TARGETS ${ISWV_INSTALL_TARGETS}
    EXPORT iswv_canopenTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY config/ DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})

configure_package_config_file(
    cmake/iswv_canopenConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/iswv_canopenConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/iswv_canopen)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/iswv_canopenConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
install(EXPORT iswv_canopenTargets
    NAMESPACE iswv::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/iswv_canopen)
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/iswv_canopenConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/iswv_canopenConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/iswv_canopen)

if(ISWV_BUILD_ROS2_TRANSPORT)
    ament_export_include_directories(include)
    ament_export_targets(iswv_canopenTargets HAS_LIBRARY_TARGET)
    ament_export_dependencies(rclcpp communication)
    ament_package()
endif()
