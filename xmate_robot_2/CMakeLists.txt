cmake_minimum_required(VERSION 3.0.2)
project(xmate_robot_2)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

catkin_package(
  #INCLUDE_DIRS include inc/include
  LIBRARIES ${PROJECT_NAME}
)

INCLUDE_DIRECTORIES(
    /usr/include/eigen3
    ${PROJECT_SOURCE_DIR}/inc
    ${PROJECT_SOURCE_DIR}/inc/include
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


LINK_DIRECTORIES(
    /usr/lib/x86_64-linux-gun
    ${PROJECT_SOURCE_DIR}/lib
    /opt/ros/kinetic/lib
)

# place_bjj_demo.cpp
add_executable(place_bjj_demo src/place_bjj_demo.cpp)
target_link_libraries(place_bjj_demo
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

# new_grasp_demo.cpp
add_executable(new_grasp_demo src/new_grasp_demo.cpp)
target_link_libraries(new_grasp_demo
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

# new_place_demo.cpp
add_executable(new_place_demo src/new_place_demo.cpp)
target_link_libraries(new_place_demo
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

# get_robot_moment.cpp
add_executable(get_robot_moment src/get_robot_moment.cpp)
target_link_libraries(get_robot_moment
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

# new_product_place_demo.cpp
add_executable(new_product_place_demo src/new_product_place_demo.cpp)
target_link_libraries(new_product_place_demo
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

# new_product_place_demo.cpp
add_executable(new_product_grasp_demo src/new_product_grasp_demo.cpp)
target_link_libraries(new_product_grasp_demo
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

# bjj_grasp_demo.cpp
add_executable(bjj_grasp_demo src/bjj_grasp_demo.cpp)
target_link_libraries(bjj_grasp_demo
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

# test_demo.cpp
add_executable(test_demo src/test_demo.cpp)
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

 add_executable(no_ar_data_test src/no_ar_data_test.cpp)
target_link_libraries(no_ar_data_test
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

 add_executable(product_ar_test src/product_ar_test.cpp)
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

add_executable(place_grasp_demo_new src/place_grasp_demo_new.cpp)
target_link_libraries(place_grasp_demo_new
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

add_executable(voice_grasp_place src/voice_grasp_place.cpp)
target_link_libraries(voice_grasp_place
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
