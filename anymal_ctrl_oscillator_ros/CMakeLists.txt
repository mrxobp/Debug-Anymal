## Set up the CMake project.
cmake_minimum_required(VERSION 3.5.1)
project(anymal_ctrl_oscillator_ros)

## Compile as C++20
set(CMAKE_CXX_STANDARD 20)

## Enable all compiler warnings and treat them as errors.
add_compile_options(-Wall -Wextra -Wpedantic)

## Export compile commands for clang.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Create a list of catkin dependencies.
set(CATKIN_PACKAGE_HEADER_DEPENDENCIES
  anymal_ctrl_oscillator
  anymal_model
  anymal_motion_control_ros
  geometry_msgs
  kindr_ros
  param_io
  roscpp
  signal_logger
)

set(CATKIN_PACKAGE_DEPENDENCIES
  ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
  message_logger
  dynamic_class_loader
  robot_control
)

## Find catkin dependencies for building and execution.
find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
)

###########
## Build ##
###########

## Declare a C++ library.
add_library(${PROJECT_NAME}
  src/OscillatorControllerRos.cpp
)

## Add CMake target dependencies of the library.
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# Specify your packages header files.
target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

# Header directories listed after 'SYSTEM' are marked as external, and don't generate warnings.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation.
install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation.
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

##########
## Test ##
##########

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
    rostest
  )

  # Run a ROS test (add rostest test-dependency).
  # The *.test file has to be on the top of the list.
  add_rostest_gtest(test_${PROJECT_NAME}
    test/ros_interface.test
    test/TestAnymalCtrlOscillatorRos.cpp
  )

  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  target_include_directories(test_${PROJECT_NAME} PRIVATE
    include
  )

  target_include_directories(test_${PROJECT_NAME} SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )

  # Generate test coverage report -> ${PROJECT_BINARY_DIR}/cmake_code_coverage/${PROJECT_NAME}/index.html
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_rostest_coverage(TEST_BUILD_TARGETS
      test_${PROJECT_NAME}
    )
  endif(cmake_code_coverage_FOUND)
endif()

#############
## Plugin ##
#############

# Build and install the plugin dependent library if robot_control_plugin exists
find_package(catkin QUIET COMPONENTS dynamic_class_loader ${CATKIN_PACKAGE_DEPENDENCIES})

if(${dynamic_class_loader_FOUND})
  add_library(${PROJECT_NAME}_plugin src/robot_control_plugin.cpp)
  target_include_directories(${PROJECT_NAME}_plugin PRIVATE
    include
  )
  target_include_directories(${PROJECT_NAME}_plugin SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
  )

  target_link_libraries(${PROJECT_NAME}_plugin
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  install(TARGETS ${PROJECT_NAME}_plugin
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(FILES robot_control_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endif()

###########
## Clang ##
###########

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
