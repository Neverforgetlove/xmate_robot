#ifndef HG_AI_ROBOT
#define HG_AI_ROBOT

#include "ini.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "move.h"
#include "control_tools.h"
#include "xmate_exception.h"

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
#include <thread>
#include <functional>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace xmate;
using namespace std;
using namespace Eigen;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
using CartesianControl = std::function<CartesianPose(RCI::robot::RobotState robot_state)>;


class HG_AI_Robot{
    public:
        HG_AI_Robot();
        ~HG_AI_Robot();
        ros::NodeHandle node;

        std::string ipaddr = "192.168.2.60";    //机器人ip
        uint16_t port = 1337;   //机器人端口
       
        void Init(xmate::Robot& robot);     //初始化函数

        double Move_Speed = 0.2; //机械臂运行速度
        float test = 0.0;
        float Ar_Pose[9][7] = {{0.0}};  //AR码信息
        float Ur_Pose[2][7] = {{0.0}};  //UR码信息
        bool RobotPower = false;    //上电状态，默认下点

        float Jog0_Robot_Moment = 3.0; //关节三默认最大力矩
        float Jog4_Robot_Moment = 25.0; //关节三默认最大力矩
        float Jog5_Robot_Moment = 25.0; //关节五默认最大力矩
        bool Start_Moment_Thread = true; //默认开启关节力矩监听线程
        bool Stop_Moment_Thread = false; //暂停力矩监听线程
        std::thread Listen_Moment; //力矩监听线程

        bool Robot_MoveJ(std::array<double, 7> target_jont, xmate::Robot& robot); //关节运动MoveJ指令
        bool Robot_MoveL(float x, float y, float z, xmate::Robot& robot); //坐标运动MoveL指令
        bool Robot_MoveR(float roation_z, xmate::Robot& robot); //末端旋转
        bool Robot_MoveX(float x, xmate::Robot& robot); //x方向移动
        bool Robot_MoveY(float y, xmate::Robot& robot); //y方向移动
        bool Robot_MoveZ(float z, xmate::Robot& robot); //z方向移动
	    bool Robot_Grasp_Control(int grasp_state, xmate::Robot& robot); //末端夹抓控制

        bool Robot_Set_Power(int power, xmate::Robot& robot); //机器人上电
        bool Robot_Set_DO(int DO_ID, int DO_State, xmate::Robot& robot); //设置DO状态
        bool Robot_Set_Speed(double New_Move_Speed, xmate::Robot& robot); //设置机械臂运行速度

        int Robot_Get_Power(xmate::Robot& robot); //获取上电状态
        int Robot_Get_DI(int DI_ID, xmate::Robot& robot); //获取DI状态
        void Robot_Get_Joint(xmate::Robot& robot); //更新机械臂关节矩阵，执行MoveJ指令后调用更新
        void Robot_Get_State(xmate::Robot& robot);//更新机械臂状态，执行MoveL指令后调用更新
        void Get_AR_Pose(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
        void Get_UR_Pose(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void Get_UR_Pose2(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void clean_ar_data();   //清除ar码信息
        void clean_ur_data();   //清除ur码信息
    private:
        RCI::robot::DOSIGNAL DO_signal; //DO变量
        RCI::robot::DISIGNAL DI_signal; //DI变量

        RCI::robot::RobotState robot_state; //机器人状态
        cart_pos current_pose, end_pose; //机器人位姿

        std::array<double, 7> current_robot_moment; //监听力矩
        std::array<double, 7> robot_joint_state; //机械臂当前关节状态
        
        /* 末端旋转角度计算 */
        Eigen::Affine3d initial_transform;
        Eigen::Affine3d rot_change;
        Eigen::Affine3d cur_transform;
        std::array<double, 16> init_position;

        const double PI = 3.14159;  //定义PI
        const int compute_moment_time = 0; //定义监听次数
	
        std::thread Listen_Moment; //力矩监听线程
        void Check_Robot_Moment(xmate::Robot& robot); //检测机械臂关节力矩
};


#endif
