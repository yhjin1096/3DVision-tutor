cmake_minimum_required(VERSION 3.0.2)
project(3DVision-tutor)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

set(OpenCV_DIR /home/cona/open_library/opencv-3.4.16/install/share/OpenCV)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES 3DVision-tutor
 CATKIN_DEPENDS roscpp
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)
link_libraries(
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_executable(Triangulation src/Triangulation.cpp)
add_executable(PoseEstimation src/PoseEstimation.cpp)
add_executable(Stella_ORBExtraction src/Stella_ORBExtraction.cpp)
add_executable(mono_vo src/mono_vo.cpp)
add_executable(stereo_vo src/stereo_vo.cpp)
add_executable(motion_only_ba src/motion_only_ba.cpp)
add_executable(comparison src/comparison.cpp)