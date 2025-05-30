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
project(vision_common)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  localization_common optimization_common 
)

# Find GTSAM
find_package(GTSAM REQUIRED)

# Find OpenCV
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/../../cmake")
find_package(OpenCV4WithXFeatures REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${GTSAM_LIBRARIES} gtsam 
  CATKIN_DEPENDS localization_common optimization_common 
)

###########
## Build ##
###########
# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(${PROJECT_NAME}
  src/brisk_feature_detector_and_matcher.cc
  src/good_features_to_track_detector.cc
  src/lk_optical_flow_feature_detector_and_matcher.cc
  src/parameter_reader.cc
  src/pose_estimation.cc
  src/spaced_feature_track.cc
  src/spaced_feature_tracker.cc
  src/surf_feature_detector_and_matcher.cc
  src/test_utilities.cc
  src/utilities.cc
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} gtsam ${OpenCV_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_feature_tracker
    test/test_feature_tracker.test
    test/test_feature_tracker.cc
  )
  target_link_libraries(test_feature_tracker
    ${PROJECT_NAME} ${catkin_LIBRARIES}  
  )
  add_rostest_gtest(test_inverse_depth_measurement
    test/test_inverse_depth_measurement.test
    test/test_inverse_depth_measurement.cc
  )
  target_link_libraries(test_inverse_depth_measurement
    ${PROJECT_NAME} ${catkin_LIBRARIES}  
  )
  add_rostest_gtest(test_lk_optical_flow_feature_detector_and_matcher
    test/test_lk_optical_flow_feature_detector_and_matcher.test
    test/test_lk_optical_flow_feature_detector_and_matcher.cc
  )
  target_link_libraries(test_lk_optical_flow_feature_detector_and_matcher
    ${PROJECT_NAME} ${catkin_LIBRARIES}  
  )
  add_rostest_gtest(test_ransac_pnp
    test/test_ransac_pnp.test
    test/test_ransac_pnp.cc
  )
  target_link_libraries(test_ransac_pnp
    ${PROJECT_NAME} ${catkin_LIBRARIES} 
  )
  add_rostest_gtest(test_reprojection_pose_estimate
    test/test_reprojection_pose_estimate.test
    test/test_reprojection_pose_estimate.cc
  )
  target_link_libraries(test_reprojection_pose_estimate
    ${PROJECT_NAME} ${catkin_LIBRARIES} 
  )
  add_rostest_gtest(test_pnp
    test/test_pnp.test
    test/test_pnp.cc
  )
  target_link_libraries(test_pnp
    ${PROJECT_NAME} ${catkin_LIBRARIES} 
  )
  add_rostest_gtest(test_pose_estimation
    test/test_pose_estimation.test
    test/test_pose_estimation.cc
  )
  target_link_libraries(test_pose_estimation
    ${PROJECT_NAME} ${catkin_LIBRARIES}  
  )
  add_rostest_gtest(test_spaced_feature_track
    test/test_spaced_feature_track.test
    test/test_spaced_feature_track.cc
  )
  target_link_libraries(test_spaced_feature_track
    ${PROJECT_NAME} ${catkin_LIBRARIES}  
  )
  add_rostest_gtest(test_spaced_feature_tracker
    test/test_spaced_feature_tracker.test
    test/test_spaced_feature_tracker.cc
  )
  target_link_libraries(test_spaced_feature_tracker
    ${PROJECT_NAME} ${catkin_LIBRARIES}  
  )
  add_rostest_gtest(test_utilities
    test/test_utilities.test
    test/test_utilities.cc
  )
  target_link_libraries(test_utilities
    ${PROJECT_NAME} ${catkin_LIBRARIES}  
  )

endif()

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
