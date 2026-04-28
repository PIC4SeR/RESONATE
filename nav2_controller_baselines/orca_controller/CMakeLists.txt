cmake_minimum_required(VERSION 3.8)
project(orca_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(FetchContent)

fetchcontent_declare(
  rvo2
  GIT_REPOSITORY https://github.com/snape/RVO2.git
  GIT_TAG        af26bedf27a84ffffb59beea996ffe2531ddc789
)

fetchcontent_makeavailable(rvo2)

# find_package(rvo2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(common_utils REQUIRED)
find_package(people_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_common REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(
  include
  /usr/local/include
)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
)

target_link_libraries(${PROJECT_NAME}
  RVO
)

set(dependencies_pkgs
  rclcpp
  nav2_core
  nav2_util
  nav2_controller
  nav2_msgs
  nav2_costmap_2d
  pluginlib
  common_utils
  people_msgs
  visualization_msgs
  geometry_msgs
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies_pkgs}
)
install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES ${PROJECT_NAME}.xml
        DESTINATION share/${PROJECT_NAME})


ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies_pkgs}
  ${rvo2_LIBRARIES}
)
pluginlib_export_plugin_description_file(nav2_core ${PROJECT_NAME}.xml)
ament_package()
