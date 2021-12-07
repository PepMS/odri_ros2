cmake_minimum_required(VERSION 3.8)
project(odri_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(odri_msgs REQUIRED)
find_package(master_board_sdk)

set(ODRI_INTERFACE_SRC src/master_board_wrapper.cpp)

include_directories(include ${MASTER_BOARD_SDK_INCLUDE_DIRS})

add_executable(master_board_wrapper ${ODRI_INTERFACE_SRC})
ament_target_dependencies(master_board_wrapper rclcpp odri_msgs)
target_link_libraries(master_board_wrapper master_board_sdk::master_board_sdk)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights uncomment
  # the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint (only
  # works in a git repo) uncomment the line when this package is not in a git
  # repo set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS master_board_wrapper DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
