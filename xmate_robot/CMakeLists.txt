cmake_minimum_required(VERSION 3.0.2)
project(xmate_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

#SET(CMAKE_BUILD_TYPE "Debug")

#message(STATUS "PRO_PATH = " ${PROJECT_SOURCE_DIR})

INCLUDE_DIRECTORIES(
    /usr/include/eigen3
    ${PROJECT_SOURCE_DIR}/inc
    ${PROJECT_SOURCE_DIR}/include
    /opt/ros/kinetic/include
    /usr/include/opencv2
    ${OpenCV_INCLUDE_DIRS}
)

find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar_msgs
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  std_msgs
  visualization_msgs
)

## System dependencies are found with CMake's conventions

LINK_DIRECTORIES(
    /usr/lib/x86_64-linux-gun
    ${PROJECT_SOURCE_DIR}/lib
    /opt/ros/kinetic/lib
)

# place_grasp_demo.cpp
add_executable(place_grasp_demo src/place_grasp_demo.cpp)
target_link_libraries(place_grasp_demo
RCI_Client.a
xMateModel.a
orocos-kdl.a
glog.a
pthread
libroscpp.so
librosconsole.so
librostime.so
libcpp_common.so
libroscpp_serialization.so
librosbag.so
librosbag_storage.so
 ${catkin_LIBRARIES})

# grasp_test_demo.cpp
add_executable(grasp_test_demo src/grasp_test_demo.cpp)
target_link_libraries(grasp_test_demo
RCI_Client.a
xMateModel.a
orocos-kdl.a
glog.a
pthread
libroscpp.so
librosconsole.so
librostime.so
libcpp_common.so
libroscpp_serialization.so
librosbag.so
librosbag_storage.so
 ${catkin_LIBRARIES})

 # place_test_demo.cpp
add_executable(place_test_demo src/place_test_demo.cpp)
target_link_libraries(place_test_demo
RCI_Client.a
xMateModel.a
orocos-kdl.a
glog.a
pthread
libroscpp.so
librosconsole.so
librostime.so
libcpp_common.so
libroscpp_serialization.so
librosbag.so
librosbag_storage.so
 ${catkin_LIBRARIES})

 # test.cpp
 add_executable(test_demo src/test.cpp)
 target_link_libraries(test_demo
 RCI_Client.a
 xMateModel.a
 orocos-kdl.a
 glog.a
 pthread
 libroscpp.so
 librosconsole.so
 librostime.so
 libcpp_common.so
 libroscpp_serialization.so
 librosbag.so
 librosbag_storage.so
  ${catkin_LIBRARIES})

 # product_place_test_demo.cpp
 add_executable(product_place_test_demo src/product_place_test_demo.cpp)
 target_link_libraries(product_place_test_demo
 RCI_Client.a
 xMateModel.a
 orocos-kdl.a
 glog.a
 pthread
 libroscpp.so
 librosconsole.so
 librostime.so
 libcpp_common.so
 libroscpp_serialization.so
 librosbag.so
 librosbag_storage.so
  ${catkin_LIBRARIES})

 # product_test_demo.cpp
 add_executable(product_test_demo src/product_test_demo.cpp)
 target_link_libraries(product_test_demo
 RCI_Client.a
 xMateModel.a
 orocos-kdl.a
 glog.a
 pthread
 libroscpp.so
 librosconsole.so
 librostime.so
 libcpp_common.so
 libroscpp_serialization.so
 librosbag.so
 librosbag_storage.so
  ${catkin_LIBRARIES})

 # product_test.cpp
 add_executable(product_ar_test src/product_test.cpp)
 target_link_libraries(product_ar_test
 RCI_Client.a
 xMateModel.a
 orocos-kdl.a
 glog.a
 pthread
 libroscpp.so
 librosconsole.so
 librostime.so
 libcpp_common.so
 libroscpp_serialization.so
 librosbag.so
 librosbag_storage.so
  ${catkin_LIBRARIES})


# place_grasp_demo_test.cpp
add_executable(place_grasp_demo_test src/place_grasp_demo_test.cpp)
target_link_libraries(place_grasp_demo_test
RCI_Client.a
xMateModel.a
orocos-kdl.a
glog.a
pthread
libroscpp.so
librosconsole.so
librostime.so
libcpp_common.so
libroscpp_serialization.so
librosbag.so
librosbag_storage.so
 ${catkin_LIBRARIES})

