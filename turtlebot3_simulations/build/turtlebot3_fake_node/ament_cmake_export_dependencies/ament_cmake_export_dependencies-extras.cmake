# generated from ament_cmake_export_dependencies/cmake/ament_cmake_export_dependencies-extras.cmake.in

set(_exported_dependencies "geometry_msgs;nav_msgs;rclcpp;sensor_msgs;tf2;tf2_msgs;turtlebot3_msgs")

find_package(ament_cmake_libraries QUIET REQUIRED)

# find_package() all dependencies
# and append their DEFINITIONS INCLUDE_DIRS, LIBRARIES, and LINK_FLAGS
# variables to turtlebot3_fake_node_DEFINITIONS, turtlebot3_fake_node_INCLUDE_DIRS,
# turtlebot3_fake_node_LIBRARIES, and turtlebot3_fake_node_LINK_FLAGS.
# Additionally collect the direct dependency names in
# turtlebot3_fake_node_DEPENDENCIES as well as the recursive dependency names
# in turtlebot3_fake_node_RECURSIVE_DEPENDENCIES.
if(NOT _exported_dependencies STREQUAL "")
  find_package(ament_cmake_core QUIET REQUIRED)
  set(turtlebot3_fake_node_DEPENDENCIES ${_exported_dependencies})
  set(turtlebot3_fake_node_RECURSIVE_DEPENDENCIES ${_exported_dependencies})
  set(_libraries)
  foreach(_dep ${_exported_dependencies})
    if(NOT ${_dep}_FOUND)
      find_package("${_dep}" QUIET REQUIRED)
    endif()
    if(${_dep}_DEFINITIONS)
      list_append_unique(turtlebot3_fake_node_DEFINITIONS "${${_dep}_DEFINITIONS}")
    endif()
    if(${_dep}_INCLUDE_DIRS)
      list_append_unique(turtlebot3_fake_node_INCLUDE_DIRS "${${_dep}_INCLUDE_DIRS}")
    endif()
    if(${_dep}_LIBRARIES)
      list(APPEND _libraries "${${_dep}_LIBRARIES}")
    endif()
    if(${_dep}_LINK_FLAGS)
      list_append_unique(turtlebot3_fake_node_LINK_FLAGS "${${_dep}_LINK_FLAGS}")
    endif()
    if(${_dep}_RECURSIVE_DEPENDENCIES)
      list_append_unique(turtlebot3_fake_node_RECURSIVE_DEPENDENCIES "${${_dep}_RECURSIVE_DEPENDENCIES}")
    endif()
    if(_libraries)
      ament_libraries_deduplicate(_libraries "${_libraries}")
      list(APPEND turtlebot3_fake_node_LIBRARIES "${_libraries}")
    endif()
  endforeach()
endif()
