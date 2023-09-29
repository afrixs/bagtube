cmake_minimum_required(VERSION 3.8)
project(bagtube_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_common REQUIRED)
find_package(bagtube_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(dependencies
  rviz_common
  rviz_rendering
  rclcpp
  bagtube_msgs
  std_srvs
)
find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

set(bagtube_rviz_plugins_headers_to_moc
  include/bagtube_rviz_plugins/bagtube_panel.hpp
)

foreach(header "${bagtube_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(bagtube_rviz_plugins_moc_files "${header}")
endforeach()

set(bagtube_rviz_plugins_source_files
  src/bagtube_panel.cpp
)

add_library(bagtube_rviz_plugins SHARED
  ${bagtube_rviz_plugins_moc_files}
  ${bagtube_rviz_plugins_source_files}
)
target_compile_features(bagtube_rviz_plugins PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(bagtube_rviz_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)
target_link_libraries(bagtube_rviz_plugins PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  ignition-math6
)
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)
ament_target_dependencies(bagtube_rviz_plugins
  PUBLIC
  ${dependencies}
)

# Export old-style CMake variables
ament_export_include_directories(include)
# Export modern CMake targets
ament_export_targets(bagtube_rviz_plugins HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${dependencies}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(bagtube_rviz_plugins PRIVATE "BAGTUBE_RVIZ_PLUGINS_BUILDING_LIBRARY")
# prevent pluginlib from using boost
target_compile_definitions(bagtube_rviz_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY icons
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS bagtube_rviz_plugins
  EXPORT bagtube_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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

ament_package(
  CONFIG_EXTRAS "bagtube_rviz_plugins-extras.cmake"
)
