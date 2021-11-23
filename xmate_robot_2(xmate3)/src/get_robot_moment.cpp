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
#include <thread>
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
//　监听力矩
std::array<double, 7> current_robot_moment;

//力矩均值
double angle1,angle2,angle3,angle4,angle5,angle6,angle7;
int run_time = 0;

// 机械臂放置到物料盘与物料台中转位置
std::array<double, 7> place_fixed_middle_pose = {{(-30.278141 * PI / 180), (-14.834426* PI / 180), (6.0488525 * PI / 180), (-74.248970 * PI / 180), (-2.8477249 * PI / 180), (-90.563409 * PI / 180), (-2.7336559 * PI / 180)}};
// 物料放置位置
std::array<double,7> pubsh_pose = {{(-95.365406 * PI / 180), (-14.531547* PI / 180), (5.96469726 * PI / 180), (-69.987455 * PI / 180), (-2.7401447 * PI / 180), (-94.285182 * PI / 180), (-4.0333042 * PI / 180)}};

// 机械臂放置物料移动到放置的中间位姿
bool robot_move_to_place_fixed_middle_pose(){
    std::array<double,7> q_init;
    //  实时获取机器人当前关节角
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.3, q_init, place_fixed_middle_pose, robot);
    //sleep(2.0);
    return true;
}

// 机械臂移动到物料放置位姿
bool robot_move_to_pubsh_pose(){
    std::array<double,7> q_init;
    //  实时获取机器人当前关节角
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.3, q_init, pubsh_pose, robot);
    //sleep(2.0);
    return true;
}

//检测机器人状态，力矩超出限制下电
void check_robot_moment()
{  
    while(1){
        current_robot_moment = robot.receiveRobotState().tau_m;
        for(auto moment:current_robot_moment){
            std::cout<<moment<<" ";
        }
	std::cout<<"----"<<std::endl;

	angle1+=current_robot_moment[0];
    angle2+=current_robot_moment[1];
	angle3+=current_robot_moment[2];
	angle4+=current_robot_moment[3];
	angle5+=current_robot_moment[4];
	angle6+=current_robot_moment[5];
	angle7+=current_robot_moment[6];
	run_time += 1;
        
        //超出就下电
	if(current_robot_moment[3]<25.0|| abs(current_robot_moment[0])>3.0){ //|| abs(current_robot_moment[0])>3.0
            std::cout<<"Power off, joint4 moment: "<<current_robot_moment[3]<<std::endl;
            robot.setMotorPower(0);
        }
	sleep(1.0);
    }
}

//向下运动
void move_down()
{
    cart_pos current_pose, end_pose;
    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] -= 0.09;
    MOVEL(0.3, current_pose, end_pose, robot);
}

//向上运动
void move_up()
{
    cart_pos current_pose, end_pose;
    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] += 0.09;
    MOVEL(0.3, current_pose, end_pose, robot);
}

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "ros_core");
    ros::NodeHandle n;
    ros::ServiceClient client;
    ros::NodeHandle node("~");

    //机械臂使能
    robot.setMotorPower(1);

    int time = 20;
    //开启线程检测
    thread t(check_robot_moment);
    t.detach();
    
    //持续做一个动作
    while(time>0){
        robot_move_to_place_fixed_middle_pose();
        sleep(1.0);
        robot_move_to_pubsh_pose();
        //check_robot_moment();
        sleep(1.0);
        move_down();
        //check_robot_moment();
        sleep(1.0);
        move_up();
        sleep(2.0);
        time -= 1;
    }
    angle1 /=run_time;
    angle2 /=run_time;
    angle3 /=run_time;
    angle4 /=run_time;
    angle5 /=run_time;
    angle6 /=run_time;
    angle7 /=run_time;
    std::cout<<angle1<<","<<angle2<<","<<angle3<<","<<angle4<<","<<angle5<<","<<angle6<<","<<angle7<<","<<run_time<<std::endl;

}
