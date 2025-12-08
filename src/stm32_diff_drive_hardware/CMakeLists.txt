cmake_minimum_required(VERSION 3.8)
project(stm32_diff_drive_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(stm32_diff_drive_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/diff_drive_hardware.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

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

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  hardware_interface
  stm32_diff_drive_msgs
)

pluginlib_export_plugin_description_file(hardware_interface stm32_diff_drive_hardware_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES stm32_diff_drive_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_libraries(${PROJECT_NAME})

ament_package()
