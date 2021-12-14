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
using namespace std;
using namespace Eigen;

using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
using CartesianControl = std::function<CartesianPose(RCI::robot::RobotState robot_state)>;

// 写死  机械臂连接
std::string ipaddr = "192.168.1.160";
uint16_t port = 1337;
xmate::Robot robot(ipaddr, port);
RCI::robot::RobotState robot_state;

const double PI = 3.14159;

// ar码pose
geometry_msgs::PoseStamped ar_traget_pose;

// ar码ID
int ar_marker_id = 0;

double robot_angle = 0.0;

std_msgs::String target_msg;

bool ready_to_grap = false;

//机械臂起始位姿
std::array<double, 7> robot_start_pose = {{0,0,PI/2,PI/2,0,PI/2,0}};

//机械臂识别位姿
std::array<double, 7> identify_ar_pose = {{0, -PI/9, 0, -PI/3.3, 0,-PI/1.865, 0}};

//机械臂抓取位姿
//std::array<double, 7> grab_pose = {{0, 0, 0, PI/2, 0,PI/2, 0}};
std::array<double,7> grab_pose = {{0,-PI/6,0,-PI/3,0,-PI/2,0}};

/*--------------------------------------------------------*/

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
    std::array<double,7> q_init;
    // std::array<double,7> q_drag = {{0,-PI/6,0,-PI/3,0,-PI/2,0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2,q_init,robot_start_pose,robot);
    return true;
}

//机械臂移动到识别位姿
bool robot_move_to_identify_ar_pose(){
    std::array<double,7> q_init;
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2,q_init,identify_ar_pose,robot);
    return true;
}

// 机械臂移动到抓取位姿
bool robot_move_to_grab_pose(){
    std::array<double,7> q_init;
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2,q_init,identify_ar_pose,robot);
    sleep(3.0);
    return true;
}

// 机械臂移动到准备位姿
bool robot_move_to_ready_pose(){
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,0,0,PI/3,0,PI/2,0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2,q_init,q_drag,robot);
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


//机械臂平移运动
bool robot_pan_move(double roation_z){

    //robot_state = robot.receiveRobotState();
    robot_state = robot.receiveRobotState();
    cart_pos current_pose, end_pose;
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    std::cout<<current_pose.pos<<std::endl;
       
    end_pose.pos[7] += ((0.114021+ar_traget_pose.pose.position.x) - 0.06); //-0.114021
    end_pose.pos[3] += ((ar_traget_pose.pose.position.y + 0.038596*cos(robot_angle)) - 0.035*cos(robot_angle));
    MOVEL(0.2, current_pose, end_pose,robot);
    sleep(1.0);

    robot_roation_move(roation_z);
    sleep(1.0);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    if(ar_traget_pose.pose.position.z!=0.0){
    	end_pose.pos[11] -= (ar_traget_pose.pose.position.z-0.120-0.008);
    }else{
    	end_pose.pos[11] -= 0.15;
    }

    MOVEL(0.2, current_pose, end_pose,robot);
    sleep(2.0);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] += 0.05;
    MOVEL(0.2, current_pose, end_pose,robot);
}

// 监听ar码回调
void ar_marker_cb(ar_track_alvar_msgs::AlvarMarkers req) {
    //ar_marker_id = count*2;
    if (!req.markers.empty() && ar_marker_id != -1) {
	int id = -1;
	for(int i=0;i<req.markers.size();i++){
            if(req.markers[i].id == ar_marker_id){
	    	id = i;
            }
        }
        float wx = req.markers[id].pose.pose.orientation.x;
        float wy = req.markers[id].pose.pose.orientation.y;
        float wz = req.markers[id].pose.pose.orientation.z;
        float ww = req.markers[id].pose.pose.orientation.w;

        float x = req.markers[id].pose.pose.position.x;
        float y = req.markers[id].pose.pose.position.y;
        float z = req.markers[id].pose.pose.position.z;
        
        // std::cout<<"orientation: "<< wx << wy << wz << ww<<std::endl;
        // std::cout<<"postion: "<< x << y <<z << std::endl;

        ar_traget_pose.pose.position=req.markers[id].pose.pose.position;
        ar_traget_pose.pose.orientation=req.markers[id].pose.pose.orientation;
        
        Eigen::Quaterniond quaternion(ww,wx,wy,wz);
        Eigen::Vector3d eulerAngle=quaternion.matrix().eulerAngles(2,1,0);
        // ZYX - RPY
        //std::cout<<"欧拉角: "<< eulerAngle<<std::endl;
        //std::cout<<"-----------"<<std::endl;
        std::cout<<"AR码角度： "<<eulerAngle(0)*57.3<<std::endl;
	    robot_angle = eulerAngle(0)*57.3;
	if(robot_angle>90.0){
	   robot_angle =180.0 - robot_angle;	
	}
        std::cout<<"末端旋转角度："<<robot_angle<<std::endl;

        //std::cout<<"ID: "<<ar_marker_id<<","<<"ar_traget"<<ar_traget_pose<<std::endl;
        
        if(ready_to_grap && (x != 0.0 || y != 0.0 || z != 0.0)){
            robot_pan_move(eulerAngle(0) - PI);
	        ready_to_grap = false;
	        sleep(2.0);
            robot_move_to_grab_pose();
            sleep(2.0);
            x = 0.0;
            y = 0.0;
            z = 0.0;
	}
    }
}

int main(int argc, char *argv[]) {

    /*ros 初始化*/
    ros::init(argc, argv, "ros_core");
    ros::NodeHandle core;

    // 监听ar码
    ros::Subscriber ar_pose = core.subscribe("ar_pose_marker", 1, ar_marker_cb);

    //robot_move_to_start();
    robot_move_to_grab_pose();
    ready_to_grap = true;
    sleep(1);

    ros::spin();
    return 0;
}