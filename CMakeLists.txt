cmake_minimum_required(VERSION 3.5)
project(twist_mux)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp              REQUIRED)
find_package(std_msgs            REQUIRED)
find_package(geometry_msgs       REQUIRED)
find_package(visualization_msgs  REQUIRED)
find_package(diagnostic_updater  REQUIRED)
find_package(rclcpp_components   REQUIRED)
find_package(rcutils             REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)




#add_library(twist_mux_diagnostics SHARED
#            src/twist_mux_diagnostics.cpp)
#
#target_compile_definitions(twist_mux_diagnostics
#  PRIVATE "COMPOSITION_BUILDING_DLL"
#  )
#
#target_include_directories(twist_mux_diagnostics
#  PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)
#
#ament_target_dependencies(twist_mux_diagnostics
#  "rclcpp"
#  "rclcpp_components"
#  "std_msgs")
#rclcpp_components_register_nodes(twist_mux_diagnostics "twist_mux::TwistMuxDiagnostics")
#set(node_plugins "${node_plugins}twist_mux::TwistMuxDiagnostics;$<TARGET_FILE:twist_mux_diagnostics_component>\n")
#
#
#
## since the package installs libraries without exporting them
## it needs to make sure that the library path is being exported
#if(NOT WIN32)
#  ament_environment_hooks(
#    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
#endif()

add_executable(twist_mux
    src/twist_mux.cpp
    src/twist_mux_node.cpp
    src/twist_mux_diagnostics.cpp
    )

target_include_directories(twist_mux
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

ament_target_dependencies(twist_mux 
     rclcpp 
     std_msgs 
     geometry_msgs 
     visualization_msgs 
     diagnostic_updater
     yaml_cpp_vendor
     )

#

install(TARGETS
  twist_mux
  DESTINATION lib/${PROJECT_NAME})


#add_executable(twist_marker
#    src/twist_marker.cpp)
#ament_target_dependencies(twist_marker rclcpp std_msgs geometry_msgs visualization_msgs diagnostic_updater) 

#install(TARGETS
#  twist_marker
#  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(
  include
)


#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # uncomment the line when a copyright and license is not present in all source files
#  #set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # uncomment the line when this package is not in a git repo
#  #set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()
#


ament_package()




#install(PROGRAMS scripts/joystick_relay.py
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#
#
#if (CATKIN_ENABLE_TESTING)
#    find_package(rostest REQUIRED)
#
#    add_rostest(test/system.test)
#endif()
