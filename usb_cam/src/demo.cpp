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

// ar码pose
geometry_msgs::PoseStamped ar_traget_pose;

// ar码ID
int ar_marker_id = -1;

// AGV巡航点
std::string move_point[3] = {"point1", "point2", "point3"};

std_msgs::String target_msg;

int count = 0;

bool ready_to_grap = false;

// 发布point给AGV
ros::Publisher agv_target_pub;

//机械臂起始位姿
std::array<double, 7> robot_start_pose = {{0,0,PI/2,PI/2,0,PI/2,0}};

//机械臂识别位姿
std::array<double, 7> identify_ar_pose = {{0, PI/4, 0, -PI/4, 0,PI/2, 0}};

//机械臂抓取位姿
//std::array<double, 7> grab_pose = {{0, 0, 0, PI/2, 0,PI/2, 0}};
std::array<double,7> grab_pose = {{0,-PI/6,0,-PI/3,0,-PI/2,0}};

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

// 控制机械臂移动
bool robot_move_to_target(){
    RCI::robot::RobotState robot_state = robot.receiveRobotState();
    cart_pos pos_start,pos_end, pos_end_base;
    pos_start.pos = robot_state.toolTobase_pos_m;

    Eigen::Matrix3d Rot = Quaternion2RotationMatrix(-0.259507, -0.000022, 0.965741, -0.00003);
    Eigen::Vector3d trans_end(-0.53555542, -0.0000489070, 0.6252839);
        
    ToArray(Rot,trans_end,pos_end.pos);
    MOVEL(0.2, pos_start, pos_end,robot);
    if (count<3){
        count ++;
    }
    return true;
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
    MOVEJ(0.2,q_init,grab_pose,robot);
    sleep(3.0);
    return true;
}

// 机械臂移动到准备位姿
bool robot_move_to_ready_pose(){
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,-PI/6,0,-PI/3,0,-PI/2,0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2,q_init,q_drag,robot);
}

//机械臂平移运动 当前位置和目标位置的差
bool robot_pan_move(){

    RCI::robot::RobotState robot_state = robot.receiveRobotState();
    robot_state = robot.receiveRobotState();
    cart_pos current_pose, end_pose;
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
       
    end_pose.pos[3] += ar_traget_pose.pose.position.x;
    end_pose.pos[7] += ar_traget_pose.pose.position.y;
    MOVEL(0.2, current_pose, end_pose,robot);
    sleep(1.0);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    if(z!=0){
    	end_pose.pos[11] -= (ar_traget_pose.pose.position.z-0.15-0.165);
    }else{
    	end_pose.pos[11] -= 0.15;
    }
    std::cout<<"here"<<std::endl;
    MOVEL(0.2, current_pose, end_pose,robot);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] += 0.15;
    MOVEL(0.2, current_pose, end_pose,robot);
}

// agv移动
void agv_move_to_next_target(int point_num){
    if(point_num<3){
        target_msg.data = move_point[point_num];
        agv_target_pub.publish(target_msg);
    }
    else{
        std::cout<<"finish!!"<<std::endl;
    }
}

// 监听ar码回调
void ar_marker_cb(ar_track_alvar_msgs::AlvarMarkers req) {
    ar_marker_id = count*2;
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

        ar_traget_pose.pose.position=req.markers[ar_marker_id].pose.pose.position;
        ar_traget_pose.pose.orientation=req.markers[ar_marker_id].pose.pose.orientation;
        
        std::cout<<"ID: "<<ar_marker_id<<","<<"ar_traget"<<ar_traget_pose<<std::endl;
        if(ready_to_grap && (ar_traget_pose.pose.position.x != 0 || ar_traget_pose.pose.position.y != 0 || ar_traget_pose.pose.position.z != 0)){
            robot_pan_move();
	    ready_to_grap = false;
	    sleep(3.0);
            robot_move_to_start();
            sleep(2.0);
            if(count<3){
                count++;
            
            if(!ready_to_grap){
               agv_move_to_next_target(count);
            }
        }
    }
}
// agv移动结果回调
void get_agv_result(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg){

    switch (msg->status.status){
        case 3:
            std::cout<<"agv move success"<<std::endl;
            sleep(2.0);
            // robot_move_to_identify_ar_pose();
            // sleep(2.0);
            robot_move_to_grab_pose();
            ready_to_grap = true;
        case 4:
            std::cout<<"agv move fail"<<std::endl;
    }
}

int main(int argc, char *argv[]) {

    /*ros 初始化*/
    ros::init(argc, argv, "ros_core");
    ros::NodeHandle core;

    // 监听ar码
    ros::Subscriber ar_pose = core.subscribe("ar_pose_marker", 1, ar_marker_cb);

    agv_target_pub = core.advertise<std_msgs::String>("/set_goal", 1);

   // 监听agv移动结果
    ros::Subscriber get_agv_res = core.subscribe("/move_base/result",1, get_agv_result);
    
    robot_move_to_start();

    sleep(1);

    target_msg.data = move_point[count];
    agv_target_pub.publish(target_msg);
    
    ros::spin();
    return 0;
}
