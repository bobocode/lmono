cmake_minimum_required(VERSION 3.1)
project(monolio)
message("$PROJECT_SOURCE_DIR")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  libviso2
  roscpp
  sensor_msgs
  nav_msgs
  message_filters
  image_transport
  cv_bridge
  image_geometry
  camera_models
  message_generation
  dynamic_reconfigure
  geometry_msgs
  message_generation
  message_runtime
  nav_msgs
  pcl_conversions
  pcl_ros
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
)

add_message_files(DIRECTORY msg FILES VisoInfo.msg)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
find_package(Gflags)
find_package(Glog)
include_directories(${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS signals thread)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

generate_messages(
  DEPENDENCIES
    std_msgs
)

add_definitions(-msse3)

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(monolio_EXT_LIBS
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
)

set(point_processor_EXT_LIBS
  ${catkin_LIBRARIES}
  ${monolio_EXT_LIBS}
)

add_library(${PROJECT_NAME}-point-processor
  src/laser_odometry/PointMapping.cc
  src/laser_odometry/PointOdometry.cc
  src/laser_odometry/PointProcessor.cc
)

add_library(${PROJECT_NAME}-visualizer
  src/visualizer/Visualizer.cc
  src/visualizer/DepthMapUtil.cc
)

add_library(${PROJECT_NAME}-initial
   src/initial/AxxbSolver.cc
   src/initial/SFM.cc
   src/initial/Solve_5pts.cc
)

set(factor_EXT_LIBS
  ${PROJECT_NAME}-initial
  ${catkin_LIBRARIES}
  ${point_processor_EXT_LIBS}
  ceres
)

add_library(${PROJECT_NAME}-factor
        src/factor/PoseLocalParameterization.cc
        src/factor/MarginalizationFactor.cc
        src/factor/MonoProjectionFactor.cc
)

add_dependencies(${PROJECT_NAME}-factor
  ${PROJECT_NAME}-initial
)

add_library(${PROJECT_NAME}-loop-detection
  src/loop_detection/KeyFrame.cc
  src/loop_detection/LoopDetector.cc
  src/loop_detection/DBoW/BowVector.cpp
  src/loop_detection/DBoW/FBrief.cpp
  src/loop_detection/DBoW/FeatureVector.cpp
  src/loop_detection/DBoW/QueryResults.cpp
  src/loop_detection/DBoW/ScoringObject.cpp
  src/loop_detection/DUtils/Random.cpp
  src/loop_detection/DUtils/Timestamp.cpp
  src/loop_detection/VocabularyBinary.cpp
  src/loop_detection/DVision/BRIEF.cpp
)

set(loop_detection_EXT_LIBS
  ${catkin_LIBRARIES}
  ${monolio_EXT_LIBS}  
)


add_library(${PROJECT_NAME}-image-process
  src/image_process/CameraModel.cc
  src/image_process/Estimator.cc
  src/image_process/FeatureManager.cc
  src/image_process/FeatureTracker.cc
  src/image_process/MeasurementManager.cc
)

set(image_process_EXT_LIBS
    ${factor_EXT_LIBS}
)

add_executable(${PROJECT_NAME}_processor_node src/processor_node.cc)
add_dependencies(${PROJECT_NAME}_processor_node ${PROJECT_NAME}-point-processor)
target_link_libraries(${PROJECT_NAME}_processor_node
  ${PROJECT_NAME}-point-processor
  ${point_processor_EXT_LIBS}
)

add_executable(${PROJECT_NAME}_odometry_node src/odometry_node.cc)
add_dependencies(${PROJECT_NAME}_odometry_node ${PROJECT_NAME}-point-processor)
target_link_libraries(${PROJECT_NAME}_odometry_node
  ${PROJECT_NAME}-point-processor
  ${point_processor_EXT_LIBS}
)

add_executable(${PROJECT_NAME}_mapping_node src/mapping_node.cc)
add_dependencies(${PROJECT_NAME}_mapping_node ${PROJECT_NAME}-point-processor)
target_link_libraries(${PROJECT_NAME}_mapping_node
  ${PROJECT_NAME}-point-processor
  ${point_processor_EXT_LIBS}
)

add_executable(${PROJECT_NAME}_estimator_node 
    src/estimator_node.cc
    src/parameter.cc                                
)
add_dependencies(${PROJECT_NAME}_estimator_node
  ${PROJECT_NAME}-image-process
  ${PROJECT_NAME}-factor
  ${PROJECT_NAME}-initial
  ${PROJECT_NAME}-point-processor
  ${PROJECT_NAME}-visualizer
)
target_link_libraries(${PROJECT_NAME}_estimator_node
  ${PROJECT_NAME}-image-process
  ${PROJECT_NAME}-factor
  ${PROJECT_NAME}-initial
  ${PROJECT_NAME}-point-processor
  ${PROJECT_NAME}-visualizer
  ${image_process_EXT_LIBS}
  pthread
)

add_executable(${PROJECT_NAME}_loop_detection_node 
    src/loop_detection_node.cc                             
)

add_dependencies(${PROJECT_NAME}_loop_detection_node
  ${PROJECT_NAME}-loop-detection
)
target_link_libraries(${PROJECT_NAME}_loop_detection_node
  ${PROJECT_NAME}-loop-detection
  ${loop_detection_EXT_LIBS}
  pthread
)




