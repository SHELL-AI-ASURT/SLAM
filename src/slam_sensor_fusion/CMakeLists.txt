cmake_minimum_required(VERSION 3.16)
project(slam_sensor_fusion VERSION 0.1.0 LANGUAGES CXX)

option(SLAM_BUILD_ROS "Build ROS 2 Humble node and message adapters" ON)
option(SLAM_ENABLE_SANITIZERS "Enable AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
option(SLAM_WARNINGS_AS_ERRORS "Treat compiler warnings as errors on project targets" ON)
option(BUILD_TESTING "Build automated unit and integration tests" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED NO_MODULE)

# Strict warning flags enforced on all project-owned code (see rules.md Section 5.1).
set(SLAM_STRICT_WARNINGS
  -Wall
  -Wextra
  -Wpedantic
  -Wshadow
  -Wconversion
  -Wnon-virtual-dtor
  -Wold-style-cast
)
if(SLAM_WARNINGS_AS_ERRORS)
  list(APPEND SLAM_STRICT_WARNINGS -Werror)
endif()

if(SLAM_ENABLE_SANITIZERS)
  set(SLAM_SANITIZER_FLAGS -fsanitize=address,undefined -fno-omit-frame-pointer)
else()
  set(SLAM_SANITIZER_FLAGS "")
endif()

# ---------------------------------------------------------------------------
# Pure C++20 / Eigen Core Library (Zero ROS dependencies)
# ---------------------------------------------------------------------------
add_library(ekf_core STATIC
  src/core/math_utils.cpp
  src/core/ekf_2d.cpp
  src/core/differential_adapter.cpp
)
add_library(slam_sensor_fusion::ekf_core ALIAS ekf_core)

target_include_directories(ekf_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(ekf_core PUBLIC Eigen3::Eigen)
target_compile_options(ekf_core PRIVATE ${SLAM_STRICT_WARNINGS} ${SLAM_SANITIZER_FLAGS})
if(SLAM_SANITIZER_FLAGS)
  target_link_options(ekf_core PUBLIC ${SLAM_SANITIZER_FLAGS})
endif()

# ---------------------------------------------------------------------------
# ROS 2 Humble Node & Adapters (Optional standalone disable for core testing)
# ---------------------------------------------------------------------------
if(SLAM_BUILD_ROS)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)

  add_library(sensor_fusion_ros STATIC
    src/ros/message_converters.cpp
    src/ros/sensor_fusion_node.cpp
  )
  target_include_directories(sensor_fusion_ros
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(sensor_fusion_ros
    PUBLIC
      ekf_core
      rclcpp::rclcpp
      tf2::tf2
      tf2_ros::tf2_ros
      tf2_geometry_msgs::tf2_geometry_msgs
      ${sensor_msgs_TARGETS}
      ${nav_msgs_TARGETS}
      ${geometry_msgs_TARGETS}
  )
  target_compile_options(sensor_fusion_ros PRIVATE ${SLAM_STRICT_WARNINGS} ${SLAM_SANITIZER_FLAGS})

  add_executable(sensor_fusion_node src/ros/main.cpp)
  target_link_libraries(sensor_fusion_node PRIVATE sensor_fusion_ros)
  target_compile_options(sensor_fusion_node PRIVATE ${SLAM_STRICT_WARNINGS} ${SLAM_SANITIZER_FLAGS})

  install(TARGETS
    ekf_core
    sensor_fusion_ros
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  install(TARGETS sensor_fusion_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY include/
    DESTINATION include
  )
  install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# ---------------------------------------------------------------------------
# Automated Tests
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  if(SLAM_BUILD_ROS)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(test_math_utils test/test_math_utils.cpp)
    target_link_libraries(test_math_utils ekf_core)
    target_compile_options(test_math_utils PRIVATE ${SLAM_STRICT_WARNINGS})

    ament_add_gtest(test_ekf_2d test/test_ekf_2d.cpp)
    target_link_libraries(test_ekf_2d ekf_core)
    target_compile_options(test_ekf_2d PRIVATE ${SLAM_STRICT_WARNINGS})

    ament_add_gtest(test_differential_adapter test/test_differential_adapter.cpp)
    target_link_libraries(test_differential_adapter ekf_core)
    target_compile_options(test_differential_adapter PRIVATE ${SLAM_STRICT_WARNINGS})

    ament_add_gtest(test_sensor_fusion_node test/test_sensor_fusion_node.cpp)
    target_link_libraries(test_sensor_fusion_node sensor_fusion_ros)
    target_compile_options(test_sensor_fusion_node PRIVATE ${SLAM_STRICT_WARNINGS})
  else()
    enable_testing()
    find_package(GTest REQUIRED)

    foreach(test_name IN ITEMS test_math_utils test_ekf_2d test_differential_adapter)
      add_executable(${test_name} test/${test_name}.cpp)
      target_link_libraries(${test_name} PRIVATE ekf_core GTest::gtest_main)
      target_compile_options(${test_name} PRIVATE ${SLAM_STRICT_WARNINGS} ${SLAM_SANITIZER_FLAGS})
      if(SLAM_SANITIZER_FLAGS)
        target_link_options(${test_name} PRIVATE ${SLAM_SANITIZER_FLAGS})
      endif()
      add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
  endif()
endif()

if(SLAM_BUILD_ROS)
  ament_package()
endif()
