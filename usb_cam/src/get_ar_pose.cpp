#include <iostream>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

void cb(ar_track_alvar_msgs::AlvarMarkers req) {
    if (!req.markers.empty()) {
        int id = req.markers[0].id;
        float wx = req.markers[0].pose.pose.orientation.x;
        float wy = req.markers[0].pose.pose.orientation.y;
        float wz = req.markers[0].pose.pose.orientation.z;
        float ww = req.markers[0].pose.pose.orientation.w;

        float x = req.markers[0].pose.pose.position.x;
        float y = req.markers[0].pose.pose.position.y;
        float z = req.markers[0].pose.pose.position.z;
        
        std::cout<<"orientation: "<< wx << wy << wz << ww<<std::endl;
        std::cout<<"postion: "<< x << y <<z << std::endl;
    }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "arlistener");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("ar_pose_marker", 1, cb);
  ros::spin();
  return 0;

}