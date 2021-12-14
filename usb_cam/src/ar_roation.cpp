#include <cmath>
#include <iostream>
#include <functional>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "ini.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "move.h"
#include "control_tools.h"

#include <ros/ros.h>
#include <ros/init.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib_msgs/GoalStatus.h>
#include <std_msgs/String.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

using namespace xmate;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
using CartesianControl = std::function<CartesianPose(RCI::robot::RobotState robot_state)>;

const double PI = 3.14159;

// 写死  机械臂连接
std::string ipaddr = "192.168.1.160";
uint16_t port = 1337;
xmate::Robot robot(ipaddr, port);

RCI::robot::RobotState robot_state;

// ar码pose
geometry_msgs::PoseStamped ar_traget_pose;

// ar码ID
int ar_marker_id = 0;

bool first_move = true;


// 四元数转旋转矩阵
Eigen::Matrix3d Quaternion2RotationMatrix(const double w,const double x,const double y,const double z)  
{  
    Eigen::Quaterniond q;  
    q.x() = x;  
    q.y() = y;  
    q.z() = z;  
    q.w() = w;  
  
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();   
    
    return R;  
}


// 机械臂移动到起始位姿
bool robot_move_to_start(){
    const double PI=3.14159;
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,PI/6,0,PI/3,0,PI/2,0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2,q_init,q_drag,robot);
    sleep(2.0);
    return true;
}

//机械臂末端转动
bool robot_roation_move(double roation_z){

    Eigen::Affine3d initial_transform;
    Eigen::Affine3d rot_change;
    Eigen::Affine3d cur_transform;

    std::array<double, 16> init_position;

    robot_state = robot.receiveRobotState();
    cart_pos current_pose,end_pose;
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos = robot_state.toolTobase_pos_m;
    init_position = robot_state.toolTobase_pos_m;

    initial_transform = Eigen::Matrix4d::Map(init_position.data()).transpose();
    
    rot_change.linear() << Eigen::AngleAxisd(roation_z,Eigen::Vector3d::UnitZ()).toRotationMatrix()*
    Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitY()).toRotationMatrix()*
    Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitX()).toRotationMatrix();

    cur_transform.linear()<<initial_transform.linear()*rot_change.linear();
    cur_transform.translation() = initial_transform.translation();
    std::array<double, 16> new_pose;
    Eigen::Map<Eigen::Matrix4d>(&new_pose[0], 4, 4) = cur_transform.matrix().transpose();

    end_pose.pos = new_pose;
    std::cout<<current_pose.pos<<std::endl;
    std::cout<<"---------------"<<std::endl;
    std::cout<<end_pose.pos<<std::endl;
    //end_pose.pos[3] += 0.02;
    MOVEL(0.2, current_pose, end_pose,robot);

}

// 监听ar码回调
void ar_marker_cb(ar_track_alvar_msgs::AlvarMarkers req) {
    //ar_marker_id = count*2;
    if (!req.markers.empty() && ar_marker_id != -1) {
        int id = req.markers[ar_marker_id].id;
        float wx = req.markers[ar_marker_id].pose.pose.orientation.x;
        float wy = req.markers[ar_marker_id].pose.pose.orientation.y;
        float wz = req.markers[ar_marker_id].pose.pose.orientation.z;
        float ww = req.markers[ar_marker_id].pose.pose.orientation.w;

        float x = req.markers[ar_marker_id].pose.pose.position.x;
        float y = req.markers[ar_marker_id].pose.pose.position.y;
        float z = req.markers[ar_marker_id].pose.pose.position.z;
        
        // std::cout<<"orientation: "<< wx << wy << wz << ww<<std::endl;
        // std::cout<<"postion: "<< x << y <<z << std::endl;
        Eigen::Quaterniond quaternion(ww,wx,wy,wz);
        Eigen::Vector3d eulerAngle=quaternion.matrix().eulerAngles(2,1,0);
        // ZYX - RPY
        std::cout<<"欧拉角: "<< eulerAngle<<std::endl;
        std::cout<<"-----------"<<std::endl;
        std::cout<<"AR码角度： "<<eulerAngle(0)*57.3<<std::endl;
        std::cout<<"末端旋转角度："<<eulerAngle(0)*57.3<<std::endl;

        /*
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));
        
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix=yawAngle*pitchAngle*rollAngle;
        */
	if(first_move){
	   robot_roation_move(eulerAngle(0));
   	   first_move = false;
	   robot_move_to_start();
    	}
    }
}



int main(int argc, char *argv[]) {

    /*ros 初始化*/
    ros::init(argc, argv, "ros_core");
    ros::NodeHandle core;

    // 监听ar码
    ros::Subscriber ar_pose = core.subscribe("ar_pose_marker", 1, ar_marker_cb);

    robot_move_to_start();
    //robot_roation_move(90/57.3);

    ros::spin();
    return 0;
}