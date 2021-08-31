cmake_minimum_required(VERSION 3.5)
project(parameter_set)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compile options set for the targets
set(PRIVATE_COMPILE_OPTIONS -Wall -Wextra -Wpedantic)

# find dependencies
find_package(absl REQUIRED status statusor)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  sensor_msgs
)

add_library(${PROJECT_NAME}
  src/parameter_set.cpp
  src/parameter_descriptor_builder.cpp
  src/validate_parameter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME} PRIVATE ${PRIVATE_COMPILE_OPTIONS})
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME} absl::status absl::statusor)

add_executable(example_node
  src/example_node/main.cpp
  src/example_node/robot_parameters.cpp
  src/example_node/robot_subsystem.cpp
)
ament_target_dependencies(example_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(example_node ${PROJECT_NAME})

# Install
install(
  TARGETS
    ${PROJECT_NAME}
    example_node
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Install include, launch, config directories
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros_testing REQUIRED)

  # Unit Tests
  ament_add_gtest(parameter_descriptor_builder_tests test/parameter_descriptor_builder_tests.cpp)
  target_link_libraries(parameter_descriptor_builder_tests ${PROJECT_NAME})

  ament_add_gmock(parameter_set_factory_tests test/parameter_set_factory_tests.cpp)
  target_link_libraries(parameter_set_factory_tests ${PROJECT_NAME})

  ament_add_gtest(set_parameters_result_builder_tests test/set_parameters_result_builder_tests.cpp)
  target_link_libraries(set_parameters_result_builder_tests ${PROJECT_NAME})

  ament_add_gtest(validate_parameter_tests test/validate_parameter_tests.cpp)
  target_link_libraries(validate_parameter_tests ${PROJECT_NAME})

  # Integration tests
  ament_add_gtest_executable(example_integration_test test/example_integration_test.cpp)
  target_link_libraries(example_integration_test ${PROJECT_NAME})
  ament_target_dependencies(example_integration_test ${THIS_PACKAGE_INCLUDE_DEPENDS})
  add_ros_test(test/example_integration_test.test.py TIMEOUT 60 ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

endif()

ament_package()
