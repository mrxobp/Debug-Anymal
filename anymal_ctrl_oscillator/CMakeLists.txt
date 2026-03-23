cmake_minimum_required(VERSION 3.10)

project(anymal_ctrl_oscillator)

# Configure C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler options
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Package dependencies
set(CATKIN_PACKAGE_DEPENDENCIES
  anymal_description
  anymal_motion_control
  anymal_model
  message_logger
  signal_logger
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

###########
## Build ##
###########

add_library(${PROJECT_NAME}
  src/OscillatorController.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

##########
## Test ##
##########

## GTest.
if(CATKIN_ENABLE_TESTING)
  ## Find catkin dependencies, including test dependencies.
  find_package(catkin REQUIRED
    COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  )
  catkin_add_gtest(test_${PROJECT_NAME}
    test/TestAnymalCtrlOscillator.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    gtest_main
    ${PROJECT_NAME}
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
    add_gtest_coverage(
      TEST_BUILD_TARGETS test_${PROJECT_NAME}
    )
  endif(cmake_code_coverage_FOUND)
endif()

###########
## Clang ##
###########

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
