#Copyright(c) 2017, United States Government, as represented by the
#Administrator of the National Aeronautics and Space Administration.
#
#All rights reserved.
#
#The Astrobee platform is licensed under the Apache License, Version 2.0
#(the "License"); you may not use this file except in compliance with the
#License.You may obtain a copy of the License at
#
#http:  // www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.See the
#License for the specific language governing permissions and limitations
#under the License.

cmake_minimum_required(VERSION 3.0)
project(graph_localizer)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# Find GTSAM
find_package(GTSAM REQUIRED)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  factor_adders
  graph_factors
  localization_common
  localization_measurements
  node_adders
  sliding_window_graph_optimizer
)

# Find OpenCV
find_package(OpenCV 4.0 REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME} ${GLOG_LIBRARIES} ${GTSAM_LIBRARIES} 
  INCLUDE_DIRS include ${GLOG_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} 
  CATKIN_DEPENDS
    factor_adders
    graph_factors
    localization_common
    localization_measurements
    node_adders
    sliding_window_graph_optimizer
)

###########
## Build ##
###########
# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB cc_files
     "src/*.cc"
)

# Declare C++ libraries
add_library(${PROJECT_NAME}
  ${cc_files}
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
