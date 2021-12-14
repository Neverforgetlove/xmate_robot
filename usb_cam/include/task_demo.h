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

using namespace std;
using namespace Eigen;

using namespace xmate;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
using CartesianControl = std::function<CartesianPose(RCI::robot::RobotState robot_state)>;

class task_demo{

    public:
        task_demo(ros::NodeHandle &n);
        ~task_demo();

        const double PI = 3.14159;

    private:


        double * update_robot_state();
        /*机械臂移动到固定位姿
           0 - 起始
           1 - point1 抓取识别
           2 - point2 抓取识别
           3 - point3 抓取识别
           4 - point4 抓取识别
           5 - 放置位姿
        */
        bool robot_move_to_pose(int pose_num);

        //机械臂末端转动
        bool robot_roation_move(double roation_z)；

        //机械臂运动到目标点
        bool robot_move_to_target(float x, float y, float z, double roation_z)；
        
        //机械臂平移运动
        bool robot_pan_move(float x = 0.0, float y = 0.0, float z = 0.0);

        //agv移动
        void agv_move_to_next_target(int point_num)；

        // 监听ar码回调
        void ar_marker_cb(ar_track_alvar_msgs::AlvarMarkers req)；

        // agv移动结果回调
        void get_agv_result(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)；


        //机械臂ip
        string ipaddr = "192.168.1.160";
        //端口号
        uint16_t port = 1337;
        //初始化机械臂实例
        xmate::Robot robot(ipaddr, port);
        RCI::robot::RobotState robot_state;

        //AR码
        geometry_msgs::PoseStamped ar_traget_pose;

        //获取机械臂关节角
        array<double,7> q_init;

        int point_num = 0;

        double robot_angle = 0.0;

        bool ready_to_grap = false;
        
        // 发布point给AGV
        ros::Publisher agv_target_pub;
        
        // ar码监听
        ros::Subscriber ar_pose;

        //agv移动结果监听
        ros::Subscriber get_agv_res；

        //发布给agv的消息
        std_msgs::String target_msg;

        //机械臂起始位姿
        array<double, 7> robot_start_pose = {{0,0,PI/2,PI/2,0,PI/2,0}};

        //机械臂识别位姿
        array<double, 7> identify_ar_pose = {{0, -PI/9, 0, -PI/3.3, 0,-PI/1.865, 0}};

        //机械臂抓取位姿
        array<double,7> grab_pose = {{0,-PI/6,0,-PI/3,0,-PI/2,0}};

        //机械臂放置位姿
        array<double,7> pubsh_pose = {{0,-PI/6,0,-PI/3,0,-PI/2,0}};
            
          //AR码ID
        int ar_marker_id[8] = [0,1,2,3,4,5,6,7,8];

        // AGV巡航点
        string move_point[4] = {"point1", "point2", "point3", "point4","point5"};

        bool agv_move = false;
        bool agv_arrive = false;

        cart_pos current_pose,end_pose;
}