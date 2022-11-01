cmake_minimum_required(VERSION 3.5)
project(tflidar_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

################### BELOW ###################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################

# catkin_package(
#   INCLUDE_DIRS include
# )

################################################################################
# Build
################################################################################

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(${AMENT_INCLUDE_DIRS} include)
# find_path(HEADERS NAMES SCSCL.h SCSerial.h SCServo.h SCS.h HINTS ${AMENT_PREFIX_PATH})

## Declare a C++ executable
add_executable(tflidar_ros_node
  src/TFlidar_ros_node.cpp
  src/TFlidar.cpp
)

## Specify libraries to link a library or executable target against
ament_target_dependencies(tflidar_ros_node
  rclcpp
  sensor_msgs
)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -std=gnu++11")

################################################################################
# Install
################################################################################

# ## Mark executables and/or libraries for installation
install(
  TARGETS tflidar_ros_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include launch src
  DESTINATION share/${PROJECT_NAME}
)

## Mark cpp header files for installation
# install(DIRECTORY include
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
# )

################### ABOVE ###################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
