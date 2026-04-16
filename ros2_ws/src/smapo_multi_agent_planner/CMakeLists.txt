cmake_minimum_required(VERSION 3.8)
project(smapo_multi_agent_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_library(smapo_planner_core
  src/planner_core.cpp
  src/task_scheduler.cpp
  src/realtime_monitor.cpp
)
target_compile_features(smapo_planner_core PUBLIC cxx_std_17)
target_include_directories(smapo_planner_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(multi_agent_planner_node
  src/multi_agent_planner_node.cpp
)
target_compile_features(multi_agent_planner_node PUBLIC cxx_std_17)
target_link_libraries(multi_agent_planner_node smapo_planner_core)
ament_target_dependencies(multi_agent_planner_node
  rclcpp
  nav_msgs
  std_msgs
  diagnostic_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS smapo_planner_core multi_agent_planner_node
  EXPORT export_smapo_planner_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_planner_core test/test_planner_core.cpp)
  target_link_libraries(test_planner_core smapo_planner_core)

  ament_add_gtest(test_task_scheduler test/test_task_scheduler.cpp)
  target_link_libraries(test_task_scheduler smapo_planner_core)

  ament_add_gtest(test_realtime_monitor test/test_realtime_monitor.cpp)
  target_link_libraries(test_realtime_monitor smapo_planner_core)
endif()

ament_export_include_directories(include)
ament_export_targets(export_smapo_planner_core HAS_LIBRARY_TARGET)
ament_package()

