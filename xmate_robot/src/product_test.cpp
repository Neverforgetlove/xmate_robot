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
#include <std_msgs/Int32.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <cmath>
#include <iostream>
#include <functional>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace xmate;
using namespace std;
using namespace Eigen;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
using CartesianControl = std::function<CartesianPose(RCI::robot::RobotState robot_state)>;


//  机械臂连接设置
std::string ipaddr = "192.168.2.60";    // 机器人ip
uint16_t port = 1337;   //  机器人端口
xmate::Robot robot(ipaddr, port);   // 机械臂连接
RCI::robot::RobotState robot_state; //  机器人状态

//  定义PI
const double PI = 3.14159;

//  参数初始化
double robot_angle = 0.0;   // 校准角度
bool ready_roation = false; // 机械臂校准
bool roation_flag = false;  //  开始校准
int identify_time = 0;  // 机械臂校准

/*--------------------------------------------------------*/
/*----------------物料抓取---------------------------*/
/*-------------------------------------------------------*/
//  机械臂固定位姿设置
// 机械臂抓取识别位姿
//std::array<double, 7> grasp_identify_pose = {{(-95.365406 * PI / 180), (-14.531547* PI / 180), (5.96469726 * PI / 180), (-69.987455 * PI / 180), (-2.7401447 * PI / 180), (-94.285182 * PI / 180), (-4.0333042 * PI / 180)}};
std::array<double, 7> grasp_identify_pose = {{(-95.365406 * PI / 180), (-14.531547* PI / 180), (5.96469726 * PI / 180), (-69.987455 * PI / 180), (-2.7401447 * PI / 180), (-94.285182 * PI / 180), (-4.0333042 * PI / 180)}};

// 机械臂放置到物料盘与物料台中转位置
std::array<double, 7> grasp_fixed_middle_pose = {{(-30.278141 * PI / 180), (-14.834426* PI / 180), (6.0488525 * PI / 180), (-74.248970 * PI / 180), (-2.8477249 * PI / 180), (-90.563409 * PI / 180), (-2.7336559 * PI / 180)}};

// 物料放置位置
std::array<double,7> grasp_id1_pubsh = {{(-18.750915 * PI / 180), (-5.1412582 * PI / 180), (3.69308166 * PI / 180), (-101.94888 * PI / 180), (-1.8495998 * PI / 180), (-72.657669 * PI / 180), (-15.904083 * PI / 180)}};
std::array<double,7> pubsh_pose = {{(-18.750915 * PI / 180), (-5.1412582 * PI / 180), (3.69308166 * PI / 180), (-101.94888 * PI / 180), (-1.8495998 * PI / 180), (-72.657669 * PI / 180), (-15.904083 * PI / 180)}};

//立体仓库放置位置 390.75244  347.643222
std::array<double,7> LTCK_pubsh_pose = {{(-91.486958 * PI / 180), (-25.924667 * PI / 180), (0.56589202 * PI / 180), (-72.387583 * PI / 180), (-1.7405090 * PI / 180), (-81.707141 * PI / 180), (3.72719764 * PI / 180)}};
std::array<double, 7> LTCK_fixed_middle_pose = {{(-18.833958 * PI / 180), (-15.501869* PI / 180), (0.74440612 * PI / 180), (-84.170942 * PI / 180), (-0.5817432 * PI / 180), (-80.469978 * PI / 180), (-9.4580955 * PI / 180)}};

/*--------------------------------------------------------*/
/*----------------物料放置---------------------------*/
/*-------------------------------------------------------*/
//机械臂识别位姿
//std::array<double, 7> place_identify_pose = {{(-112.415583 * PI / 180), (-10.4204635 * PI / 180), (48.4544586 * PI / 180), (-67.0699264 * PI / 180), (-6.84177017 * PI / 180), (-99.5314127 * PI / 180), (13.6344623 * PI / 180)}};
std::array<double, 7> place_identify_pose = {{(-95.365406 * PI / 180), (-14.531547* PI / 180), (5.96469726 * PI / 180), (-69.987455 * PI / 180), (-2.7401447 * PI / 180), (-94.285182 * PI / 180), (-4.0333042 * PI / 180)}};

std::array<double, 7> place_fixed_middle_pose = {{(-30.278141 * PI / 180), (-14.834426* PI / 180), (6.0488525 * PI / 180), (-74.248970 * PI / 180), (-2.8477249 * PI / 180), (-90.563409 * PI / 180), (-2.7336559 * PI / 180)}};


// 机械臂移动到物料放置位姿
bool robot_move_to_pubsh_pose(){
    std::array<double,7> q_init;
    //  实时获取机器人当前关节角
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.3, q_init, pubsh_pose, robot);
    //sleep(2.0);
    return true;
}


//机械臂移动到立体仓库放置位姿
bool robot_move_to_LTCK_pubsh_pose(){
    std::array<double,7> q_init;
    //  实时获取机器人当前关节角
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.3, q_init, LTCK_pubsh_pose, robot);
    sleep(2);
    return true;
}


//机械臂移动到立体仓库放置中间位姿
bool robot_move_to_LTCK_fixed_middle_pose(){
    std::array<double,7> q_init;
    //  实时获取机器人当前关节角
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.3, q_init, LTCK_fixed_middle_pose, robot);
    sleep(2.0);
    return true;
}


int main(int argc, char *argv[]) {
    /*ros 初始化*/
    ros::init(argc, argv, "ros_core");
    ros::NodeHandle n;
    ros::ServiceClient client;
    ros::NodeHandle node("~");


   //   机械臂使能
    robot.automaticErrorRecovery();

/*--------------------------------------------------------*/
/*----------------成品放置到立体仓库---------------------------*/
/*-------------------------------------------------------*/
    //  运动到中间位置
    robot_move_to_LTCK_fixed_middle_pose();

    //  运动到识别位置
    robot_move_to_LTCK_pubsh_pose();

    // //  回到中间位置
    // robot_move_to_LTCK_fixed_middle_pose();

    // 回到起始位置
    robot_move_to_pubsh_pose();

    ros::spin();
    return 0;

}
