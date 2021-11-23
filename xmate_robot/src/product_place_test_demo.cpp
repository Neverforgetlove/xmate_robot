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

//  抓取物料偏移量
double grasp_x, grasp_y, grasp_z;
//  放置物料偏移量
double place_x, place_y, place_z;
//  放置物料偏移量
double sheet_x, sheet_y, sheet_z;

//  将id1物料放置到物料盘上偏移量
double graspid1_x, graspid1_y, graspid1_z;
//  将id2物料放置到物料盘上偏移量
double graspid2_x, graspid2_y, graspid2_z;
//  将id1物料放置到装配台上偏移量
double placeid1_x, placeid1_y, placeid1_z;
//  将id2物料放置到装配台上偏移量
double placeid2_x, placeid2_y, placeid2_z;

//  将成品放置到物料盘上
double product_x, product_y, product_z;
//  从物料盘上抓取成品
double product_grasp_x, product_grasp_y, product_grasp_z; 

//  将成品放置到立体仓库
double Ste_warehouse_x, Ste_warehouse_y, Ste_warehouse_z; 

double arcodeid; 

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

/*--------------------------------------------------------*/

/*--------------------------------------------------------*/
/*----------------获取AGV的flag---------------------------*/
/*-------------------------------------------------------*/
class AGVFlag{
public:
    int count = 0;
    int flag;
public:
    void agv_callback(const std_msgs::Int32::ConstPtr& msg);
    int print_agvflag(){
        return flag;
    }
};

//   AGV导航Flag
void AGVFlag::agv_callback(const std_msgs::Int32::ConstPtr& msg)
{
    flag = msg -> data;
    print_agvflag();
    ++count;
}
/*--------------------------------------------------------*/

/*----------------获取装配台的flag---------------------------*/
/*-------------------------------------------------------*/
class AssStandFlag{
public:
    int count = 0;
    int flag;
public:
    void AssStand_callback(const std_msgs::Int32::ConstPtr& msg);
    int print_AssStandFlag(){
        return flag;
    }
};

//   AGV导航Flag
void AssStandFlag::AssStand_callback(const std_msgs::Int32::ConstPtr& msg)
{
    flag = msg -> data;
    print_AssStandFlag();
    ++count;
}

/*--------------------------------------------------------*/
/*----------------获取AR码的pose---------------------------*/
/*-------------------------------------------------------*/
class ARPose{
public:
    int count = 0;
    float x, y, z, ww, wx, wy, wz;
public:
    void callbackid0(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void callbackid1(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void callbackid2(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void callbackid3(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void callbackid4(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void callbackid5(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void callbackid6(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void callbackid7(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
    void callbackid8(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);

    float print_datax(){
        return x;
    }

    float print_datay(){
        return y;
    }

    float print_dataz(){
        return z;
    }

    float print_dataww(){
        return ww; 
    }

    float print_datawx(){
        return wx;
    }

    float print_datawy(){
        return wy;
    }

    float print_datawz(){
        return wz;
    }
};

//  AR码id为0
void ARPose::callbackid0(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    int id = 0;
    if (!msg->markers.empty()) {
        for(int i=0;i<msg->markers.size();i++){
            if(msg->markers[i].id == id){
 	    	    id = i;
                //  位置
                x = msg->markers[id].pose.pose.position.x;
                y = msg->markers[id].pose.pose.position.y;
                z = msg->markers[id].pose.pose.position.z;
                //  四元数
                ww = msg->markers[id].pose.pose.orientation.w;
                wx = msg->markers[id].pose.pose.orientation.x;
                wy = msg->markers[id].pose.pose.orientation.y;
                wz = msg->markers[id].pose.pose.orientation.z;
                //  位置
                print_datax();
                print_datay();
                print_dataz();
                //  四元数
                print_dataww();
                print_datawx();
                print_datawy();
                print_datawz();
                ++count;   
             }
        }
    }

}

//  AR码id为1
void ARPose::callbackid1(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    int id = 1;
    if (!msg->markers.empty()) {
        for(int i=0;i<msg->markers.size();i++){
            if(msg->markers[i].id == id){
 	    	    id = i;
                //  位置
                x = msg->markers[id].pose.pose.position.x;
                y = msg->markers[id].pose.pose.position.y;
                z = msg->markers[id].pose.pose.position.z;
                //  四元数
                ww = msg->markers[id].pose.pose.orientation.w;
                wx = msg->markers[id].pose.pose.orientation.x;
                wy = msg->markers[id].pose.pose.orientation.y;
                wz = msg->markers[id].pose.pose.orientation.z;
                //  位置
                print_datax();
                print_datay();
                print_dataz();
                //  四元数
                print_dataww();
                print_datawx();
                print_datawy();
                print_datawz();
                ++count;   

             }
        }
    }

}

//  AR码id为2
void ARPose::callbackid2(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    int id = 2;
    if (!msg->markers.empty()) {
        for(int i=0;i<msg->markers.size();i++){
            if(msg->markers[i].id == id){
 	    	    id = i;
                //  位置
                x = msg->markers[id].pose.pose.position.x;
                y = msg->markers[id].pose.pose.position.y;
                z = msg->markers[id].pose.pose.position.z;
                //  四元数
                ww = msg->markers[id].pose.pose.orientation.w;
                wx = msg->markers[id].pose.pose.orientation.x;
                wy = msg->markers[id].pose.pose.orientation.y;
                wz = msg->markers[id].pose.pose.orientation.z;
                //  位置
                print_datax();
                print_datay();
                print_dataz();
                //  四元数
                print_dataww();
                print_datawx();
                print_datawy();
                print_datawz();
                ++count;   

             }
        }  
    }
}

//  AR码id为3
void ARPose::callbackid3(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    int id = 3;
    if (!msg->markers.empty()) {
        for(int i=0;i<msg->markers.size();i++){
            if(msg->markers[i].id == id){
 	    	    id = i;
                //  位置
                x = msg->markers[id].pose.pose.position.x;
                y = msg->markers[id].pose.pose.position.y;
                z = msg->markers[id].pose.pose.position.z;
                //  四元数
                ww = msg->markers[id].pose.pose.orientation.w;
                wx = msg->markers[id].pose.pose.orientation.x;
                wy = msg->markers[id].pose.pose.orientation.y;
                wz = msg->markers[id].pose.pose.orientation.z;
                //  位置
                print_datax();
                print_datay();
                print_dataz();
                //  四元数
                print_dataww();
                print_datawx();
                print_datawy();
                print_datawz();
                ++count;   
             }
        }

    }
}

//  AR码id为4
void ARPose::callbackid4(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    int id = 4;
    if (!msg->markers.empty()) {
        for(int i=0;i<msg->markers.size();i++){
            if(msg->markers[i].id == id){
 	    	    id = i;
                //  位置
                x = msg->markers[id].pose.pose.position.x;
                y = msg->markers[id].pose.pose.position.y;
                z = msg->markers[id].pose.pose.position.z;
                //  四元数
                ww = msg->markers[id].pose.pose.orientation.w;
                wx = msg->markers[id].pose.pose.orientation.x;
                wy = msg->markers[id].pose.pose.orientation.y;
                wz = msg->markers[id].pose.pose.orientation.z;
                //  位置
                print_datax();
                print_datay();
                print_dataz();
                //  四元数
                print_dataww();
                print_datawx();
                print_datawy();
                print_datawz();
                ++count;                    
             }
        }

    }
}

//  AR码id为5
void ARPose::callbackid5(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    int id = 5;
    if (!msg->markers.empty()) {
        for(int i=0;i<msg->markers.size();i++){
            if(msg->markers[i].id == id){
 	    	    id = i;
                //  位置
                x = msg->markers[id].pose.pose.position.x;
                y = msg->markers[id].pose.pose.position.y;
                z = msg->markers[id].pose.pose.position.z;
                //  四元数
                ww = msg->markers[id].pose.pose.orientation.w;
                wx = msg->markers[id].pose.pose.orientation.x;
                wy = msg->markers[id].pose.pose.orientation.y;
                wz = msg->markers[id].pose.pose.orientation.z;
                //  位置
                print_datax();
                print_datay();
                print_dataz();
                //  四元数
                print_dataww();
                print_datawx();
                print_datawy();
                print_datawz();
                ++count;                    
             }
        }       
    }
}

//  AR码id为6
void ARPose::callbackid6(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    int id = 6;
    if (!msg->markers.empty()) {
        for(int i=0;i<msg->markers.size();i++){
            if(msg->markers[i].id == id){
 	    	    id = i;
                //  位置
                x = msg->markers[id].pose.pose.position.x;
                y = msg->markers[id].pose.pose.position.y;
                z = msg->markers[id].pose.pose.position.z;
                //  四元数
                ww = msg->markers[id].pose.pose.orientation.w;
                wx = msg->markers[id].pose.pose.orientation.x;
                wy = msg->markers[id].pose.pose.orientation.y;
                wz = msg->markers[id].pose.pose.orientation.z;
                //  位置
                print_datax();
                print_datay();
                print_dataz();
                //  四元数
                print_dataww();
                print_datawx();
                print_datawy();
                print_datawz();
                ++count;                    
             }
        }      
    }

}

//  AR码id为7
void ARPose::callbackid7(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    int id = 7;
    if (!msg->markers.empty()) {
        for(int i=0;i<msg->markers.size();i++){
            if(msg->markers[i].id == id){
 	    	    id = i;
                //  位置
                x = msg->markers[id].pose.pose.position.x;
                y = msg->markers[id].pose.pose.position.y;
                z = msg->markers[id].pose.pose.position.z;
                //  四元数
                ww = msg->markers[id].pose.pose.orientation.w;
                wx = msg->markers[id].pose.pose.orientation.x;
                wy = msg->markers[id].pose.pose.orientation.y;
                wz = msg->markers[id].pose.pose.orientation.z;
                //  位置
                print_datax();
                print_datay();
                print_dataz();
                //  四元数
                print_dataww();
                print_datawx();
                print_datawy();
                print_datawz();
                ++count;                    
             }
        }
    }
}

//  AR码id为8
void ARPose::callbackid8(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    int id = 8;
    if (!msg->markers.empty()) {
        for(int i=0;i<msg->markers.size();i++){
            if(msg->markers[i].id == id){
 	    	    id = i;
                //  位置
                x = msg->markers[id].pose.pose.position.x;
                y = msg->markers[id].pose.pose.position.y;
                z = msg->markers[id].pose.pose.position.z;
                //  四元数
                ww = msg->markers[id].pose.pose.orientation.w;
                wx = msg->markers[id].pose.pose.orientation.x;
                wy = msg->markers[id].pose.pose.orientation.y;
                wz = msg->markers[id].pose.pose.orientation.z;
                //  位置
                print_datax();
                print_datay();
                print_dataz();
                //  四元数
                print_dataww();
                print_datawx();
                print_datawy();
                print_datawz();
                ++count;                    
             }
        }

    }
}
/*-------------------------------------------------------*/

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

//  吸盘动作
bool test_DI(int di1){
    int timeout = 3;
    if(di1 == 0){
        RCI::robot::DISIGNAL DI_signal_0 = RCI::robot::DISIGNAL::DI1_0;
            bool DI_State = robot.getDI(DI_signal_0);
            if(!DI_State){
                while(DI_State != true && timeout>0){
                    DI_State = robot.getDI(DI_signal_0);
                    if(DI_State){
                        return true;
                    }
                    sleep(1.0);
                    timeout -=1;
                }
                return false;
            }
            else{
                return true;
            }

    }
    else if(di1 == 1){
        RCI::robot::DISIGNAL DI_signal_1 = RCI::robot::DISIGNAL::DI1_1;
            bool DI_State1 = robot.getDI(DI_signal_1);
            if(!DI_State1){
                while(DI_State1 != true && timeout>0){
                    DI_State1 = robot.getDI(DI_signal_1);
                    if(DI_State1){
                        return true;
                    }
                    sleep(1.0);
                    timeout -=1;
                }
                return false;
            }
            else{
                return true;
            }
    }
    else{
        return false;
    }
}

// do1_0 - 夹爪闭合  do1_1 开
// di1_0   闭合成功    di1_1 开成功 
// 参数０：闭合　　参数１：开
bool test_DO(int do1){

    if(do1 == 0){
            RCI::robot::DOSIGNAL DO_signal = RCI::robot::DOSIGNAL::DO0_0;
	    RCI::robot::DOSIGNAL DO_signal_1 = RCI::robot::DOSIGNAL::DO0_3;
            robot.setDO(DO_signal_1,true);
	    robot.setDO(DO_signal,true);
            sleep(1.0);
            if(test_DI(do1)){
                sleep(1.0);
                cout<<"close success"<<endl;
                robot.setDO(DO_signal,false);
                robot.setDO(DO_signal_1,false);
            }else{
                robot.setDO(DO_signal,false);
                robot.setDO(DO_signal_1,false);
            }
            return true;
    }
    else if(do1 == 1){
        RCI::robot::DOSIGNAL DO_signal = RCI::robot::DOSIGNAL::DO0_1;
        RCI::robot::DOSIGNAL DO_signal_1 = RCI::robot::DOSIGNAL::DO0_3;
            robot.setDO(DO_signal,true);
            robot.setDO(DO_signal_1,true);
            sleep(1.0);
            if(test_DI(do1)){
                sleep(1.0);
		cout<<"open success"<<endl;
                robot.setDO(DO_signal,false);
                robot.setDO(DO_signal_1,false);
            }else{
                robot.setDO(DO_signal,false);
                robot.setDO(DO_signal_1,false);
            }
            return true;
    }
    else{
        return false;
    }
}
//机械臂移动到物料抓取识别位姿
bool robot_move_to_grasp_identify_pose(){
    std::array<double,7> q_init;

    //  实时获取机器人当前关节角
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.3, q_init, grasp_identify_pose, robot);
    roation_flag = true;
    //sleep(2);
    return true;
}

//机械臂移动到物料放置识别位姿
bool robot_move_to_place_identify_pose(){
    std::array<double,7> q_init;
    //  实时获取机器人当前关节角
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.3, q_init, place_identify_pose, robot);
    roation_flag = true;
    //sleep(2);
    return true;
}


// 机械臂抓取物料移动到放置的中间位姿
bool robot_move_to_grasp_fixed_middle_pose(){
    std::array<double,7> q_init;
    //  实时获取机器人当前关节角
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.3, q_init, grasp_fixed_middle_pose, robot);
    //sleep(2.0);
    return true;
}

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

// 机械臂移动到物料放置位姿
bool robot_move_to_grasp_id1_pubsh(){
    std::array<double,7> q_init;
    //  实时获取机器人当前关节角
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.3, q_init, grasp_id1_pubsh, robot);
    //sleep(2.0);
    return true;
}


// //机械臂末端转动
bool robot_roation_move(double roation_z){
    Eigen::Affine3d initial_transform;
    Eigen::Affine3d rot_change;
    Eigen::Affine3d cur_transform;

    std::array<double, 16> init_position;
    //  实时获取机器人当前关节角
    robot_state = robot.receiveRobotState();
    cart_pos current_pose,end_pose;
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos = robot_state.toolTobase_pos_m;
    init_position = robot_state.toolTobase_pos_m;
    initial_transform = Eigen::Matrix4d::Map(init_position.data()).transpose();
    
    //  旋转顺序ZYX
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
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(1.0);
    return true;
}

//机械臂抓取物料校准前平移运动
bool grasp_move_xy(float& x, float& y){
    /*                   转动                                   */
    //robot_state = robot.receiveRobotState();
    //  实时获取机器人当前关节角
    robot_state = robot.receiveRobotState();
    cart_pos current_pose, end_pose;
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    cout<<end_pose.pos<<endl;

    //  x=x',x'为相机的x方向的坐标
    end_pose.pos[3] += ((x) - 0.05); //0.022 增大往左
    //  y方向,y = -y',y'为相机y方向的坐标
    end_pose.pos[7] += ((-y) + 0.020);  // 0.035

    cout<<end_pose.pos<<endl;
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(1.0);
    return true;
}

//机械臂抓取物料校准后平移运动
bool grasp_ar_xy(float& x, float& y){
    /*                   转动                                   */
    //robot_state = robot.receiveRobotState();
    //  实时获取机器人当前关节角
    robot_state = robot.receiveRobotState();
    cart_pos current_pose, end_pose;
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    cout<<end_pose.pos<<endl;

    //  x=x',x'为相机的x方向的坐标
    end_pose.pos[3] += ((x));
    //  y方向,y = -y',y'为相机y方向的坐标
    end_pose.pos[7] += ((-y));

    cout<<end_pose.pos<<endl;
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(1.0);
    return true;
}

//机械臂抓取物料动作
bool grasp_move_xyz(float& x, float& y, float& z, double& grasp_x, double& grasp_y, double& grasp_z){
    /*                   转动                          -0.08)         */
    //robot_state = robot.receiveRobotState();
    //  实时获取机器人当前关节角
    robot_state = robot.receiveRobotState();
    cart_pos current_pose, end_pose;
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;

    // 近到远
    //  x=x',x'为相机的x方向的坐标
    end_pose.pos[3] += ((x) - grasp_x); //增大往左(默认:0.007)
    //  y方向,y = -y',y'为相机y方向的坐标
    end_pose.pos[7] += ((-y) + grasp_y);  // 增大向前(默认:0.07)
    // z方向
    end_pose.pos[11]  -= (z - grasp_z); // 往下减小(默认:0.223)

    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(1.0);
    return true;
}

//机械臂平移运动
bool place_move_xy(float& x, float& y){
    /*                   转动                                   */
    //robot_state = robot.receiveRobotState();
    //  实时获取机器人当前关节角
    robot_state = robot.receiveRobotState();
    cart_pos current_pose, end_pose;
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    cout<<end_pose.pos<<endl;

    //  x=x',x'为相机的x方向的坐标
    end_pose.pos[3] += ((x));
    //  y方向,y = -y',y'为相机y方向的坐标
    end_pose.pos[7] += ((-y));

    cout<<end_pose.pos<<endl;
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(1.0);
    return true;
}

//机械臂平移运动
bool place_ar_move_xy(float& x, float& y){
    /*                   转动                          -0.08)         */
    //robot_state = robot.receiveRobotState();
    //  实时获取机器人当前关节角
    robot_state = robot.receiveRobotState();
    cart_pos current_pose, end_pose;
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;

    // 近到远
    //  x=x',x'为相机的x方向的坐标
    end_pose.pos[3] += ((x) - 0.024); //0.022 增大往左
    //  y方向,y = -y',y'为相机y方向的坐标
    end_pose.pos[7] += ((-y) + 0.020);  // 0.035
    // // z方向
    // end_pose.pos[11]  -= (z - 0.253); // 向上变大

    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(1.0);
    return true;
}

//机械臂平移运动
bool place_move_xyz(float& x, float& y, float& z, double& place_x, double& place_y, double& place_z){
    /*                   转动                          -0.08)         */
    //robot_state = robot.receiveRobotState();
    //  实时获取机器人当前关节角
    robot_state = robot.receiveRobotState();
    cart_pos current_pose, end_pose;
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;

    // 近到远
    //  x=x',x'为相机的x方向的坐标
    end_pose.pos[3] += ((x) - place_x); //增大往左(默认：0.007)
    //  y方向,y = -y',y'为相机y方向的坐标
    end_pose.pos[7] += ((-y) + place_y);  // 增大向前(默认：0.053)
    // z方向
    end_pose.pos[11]  -= (z - place_z); // 往下增大(默认：0.253)

    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(1.0);
    return true;
}

//  钣金件抓取
//机械臂平移运动
bool grasp_sheet_metal_xy(float& x, float& y, float& z, double& sheet_x, double& sheet_y, double& sheet_z){
    /*                   转动                          -0.08)         */
    robot_state = robot.receiveRobotState();
    // 实时获取机器人当前关节角
    robot_state = robot.receiveRobotState();
    cart_pos current_pose, end_pose;
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;

    // 近到远
    //  x=x',x'为相机的x方向的坐标
    end_pose.pos[3] += ((x) + 0.234); //0.022 增大往左
    //  y方向,y = -y',y'为相机y方向的坐标
    end_pose.pos[7] += ((-y) + 0.053);  // 0.035增大向前

    MOVEL(0.3, current_pose, end_pose, robot);
}


bool grasp_sheet_metal_xyz(float& x, float& y, float& z, double& sheet_x, double& sheet_y, double& sheet_z){

    //  实时获取机器人当前关节角
    robot_state = robot.receiveRobotState();
    cart_pos current_pose, end_pose;
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    
   // x方向
    end_pose.pos[3] +=  (x - sheet_x);
    //  y方向
    end_pose.pos[7] += ((-y) - sheet_y);  // 增大向前(默认： 0.093)

    MOVEL(0.3, current_pose, end_pose, robot);

    //  实时获取机器人当前关节角
    robot_state = robot.receiveRobotState();
    //cart_pos current_pose, end_pose;
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    
    // z方向
    end_pose.pos[11]  -= (z - sheet_z); // 往下增大(默认： 0.253)

    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(1.0);
    return true;
}


//  抓取id1物料放置到物料盘上
bool grasp_push_actionid1(double& graspid1_x, double& graspid1_y, double& graspid1_z){
    cart_pos current_pose, end_pose;

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[3] -= graspid1_x; // 减小往左(默认:0.011)
    end_pose.pos[7] -= graspid1_y; // 增大往左(默认:0.0)
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(1.0);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] -= graspid1_z; // 增大往下(默认:0.060134663)
    MOVEL(0.3, current_pose, end_pose, robot);

    //sleep(2.0);
    test_DO(1);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] += 0.043134663;
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(2.0);
    return true;
}

//  抓取成品放置到物料盘上
bool push_product(double& product_x, double& product_y, double& product_z){
    cart_pos current_pose, end_pose;

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[3] -= product_x; // 减小往左(默认:0.011)
    end_pose.pos[7] -= product_y; // 增大往左(默认:0.0)
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(1.0);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] -= product_z; // 增大往下(默认:0.060134663)
    MOVEL(0.3, current_pose, end_pose, robot);

    //sleep(2.0);
    test_DO(1);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] += 0.043134663;
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(2.0);
    return true;
}


//  抓取id2物料放置到物料盘上
bool grasp_push_actionid2(double& graspid2_x, double& graspid2_y, double& graspid2_z){
    cart_pos current_pose, end_pose;

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[3] -= graspid2_x; // 增大往左(默认：0.0)
    end_pose.pos[7] -= graspid2_y; // 增大往前(默认:0.11)
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(1.0);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] -= graspid2_z; // 增大往下(默认:0.046834663)
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(2.0);
    test_DO(1);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] += 0.043134663;
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(2.0);
    return true;
}

//  放置id1物料到装配台上
bool place_push_actionid1(double& placeid1_x, double& placeid1_y, double& placeid1_z){
    cart_pos current_pose, end_pose;

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[3] -= placeid1_x;   // 增大往左(默认:0.015)
    end_pose.pos[7] += placeid1_y;   // 增大往里(默认:0.004)
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(1.0);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] -= placeid1_z;  // 增大往下(默认:0.040134663)
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(2.0);
    test_DO(0);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] += 0.043134663;
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(2.0);

    return true;
}

//  放置id1物料到装配台上
bool place_push_actionid2(double& placeid2_x, double& placeid2_y, double& placeid2_z){
    cart_pos current_pose, end_pose;

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[3] -= placeid2_x; // 增大往左(默认:0.0)
    end_pose.pos[7] -= placeid2_y; // 增大往里(默认:0.1115)
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(1.0);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] -= placeid2_z; // 增大往下(默认:0.041234663)
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(2.0);
    test_DO(0);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] += 0.043134663;
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(2.0);
    return true;
}

//放置动作
bool robot_push_action_z(){
    cart_pos current_pose, end_pose;
    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] += 0.043134663;
    MOVEL(0.3, current_pose, end_pose, robot);
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

//成品放置动作
bool cp_robot_push_action(){
    cart_pos current_pose, end_pose;
    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] -= 0.043109218;
    MOVEL(0.3, current_pose, end_pose, robot);
    sleep(1.0);
    test_DO(1);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] += 0.04;
    MOVEL(0.3, current_pose, end_pose, robot);
    sleep(1.0);
   
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

//  放置id1物料到装配台上
bool product_grasp_actionid(double& product_grasp_x, double& product_grasp_y, double& product_grasp_z){
    cart_pos current_pose, end_pose;

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[3] -= product_grasp_x;   // 增大往左(默认:0.015)
    end_pose.pos[7] += product_grasp_y;   // 增大往里(默认:0.004)
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(1.0);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] -= product_grasp_z;  // 增大往下(默认:0.040134663)
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(2.0);
    test_DO(0);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] += 0.043134663;
    MOVEL(0.3, current_pose, end_pose, robot);
    //sleep(2.0);

    return true;
}



int main(int argc, char *argv[]) {
    //  初始化参数
    float sx, sy, sz, ww, wx, wy, wz;
    float sx0, sy0, sz0, ww0, wx0, wy0, wz0;
    //  机械臂平移运动
    float sx1, sy1, sz1, ww1, wx1, wy1, wz1;
    //  机械臂平移运动校准
    float sx2, sy2, sz2, ww2, wx2, wy2, wz2;
    //  机械臂平移运动放置
    float sx3, sy3, sz3, ww3, wx3, wy3, wz3;

    //  成品放置
    float sx4, sy4, sz4, ww4, wx4, wy4, wz4;
    float sx5, sy5, sz5, ww5, wx5, wy5, wz5;

    //  机械臂平移运动放置
    int arFlag = 0;

    int flag;
    // int arcodeid = 0;    

    std_msgs::Int32 tag;
    /*ros 初始化*/
    ros::init(argc, argv, "ros_core");
    ros::NodeHandle n;
    ros::ServiceClient client;
    ros::NodeHandle node("~");

    bool roation_flag = true;

    // 设置抓取物料参数
    node.param("grasp_x", grasp_x, 0.007);  // 抓取物料时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("grasp_y", grasp_y, 0.07);  //　抓取物料时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("grasp_z", grasp_z, 0.223);    //　抓取物料时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小

    // 设置放置物料参数
    node.param("place_x", place_x, 0.007);  // 放置物料到装配台时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左(默认：0.007)
    node.param("place_y", place_y, 0.053);  //　放置物料到装配台时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前(默认：0.053)
    node.param("place_z", place_z, 0.253);    //　放置物料到装配台时机械臂的z坐标偏移量(笛卡尔坐标系)，往下增大(默认：0.253)

    // 设置放置成品参数
    node.param("sheet_x", sheet_x, -0.020);  // 抓取成品时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前(默认： 0.093)   
    node.param("sheet_y", sheet_y, 0.093);  // 抓取成品时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前(默认： 0.093)
    node.param("sheet_z", sheet_z, 0.253);  //　抓取成品时机械臂的z坐标偏移量(笛卡尔坐标系)，往下增大(默认： 0.253)

    // 物料id1在物料盘上面放置参数
    node.param("graspid1_x", graspid1_x, 0.011);  //  抓取id1物料到物料盘时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("graspid1_y", graspid1_y, 0.0);  //　抓取id1物料到物料盘时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("graspid1_z", graspid1_z, 0.060134663);  //　抓取id1物料到物料盘时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小

    // 物料id2在物料盘上面放置参数
    node.param("graspid2_x", graspid2_x, 0.0);  //  抓取id2物料到物料盘时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("graspid2_y", graspid2_y, 0.11);  //　抓取id2物料到物料盘时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("graspid2_z", graspid2_z, 0.046834663);  //　抓取id2物料到物料盘时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小

    // 物料id1从物料盘上面抓取参数
    node.param("placeid1_x", placeid1_x, 0.015);  //  从物料盘抓取id1物料时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("placeid1_y", placeid1_y, 0.004);  //　从物料盘抓取id1物料时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("placeid1_z", placeid1_z, 0.040134663);  //　从物料盘抓取id1物料时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小

    // 物料id2从物料盘上面抓取参数
    node.param("placeid2_x", placeid2_x, 0.0);  //  从物料盘抓取id2物料时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("placeid2_y", placeid2_y, 0.1115);  //　从物料盘抓取id2物料时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("placeid2_z", placeid2_z, 0.041234663);  //　从物料盘抓取id2物料时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小

    // 成品放置到物料盘上
    node.param("product_x", product_x, 0.011);  //  成品放置到物料盘上的x坐标偏移量(笛卡尔坐标系)
    node.param("product_y", product_y, 0.0);  //　成品放置到物料盘上的y坐标偏移量(笛卡尔坐标系)
    node.param("product_z", product_z, 0.060134663);  //　成品放置到物料盘上的z坐标偏移量(笛卡尔坐标系)

    // 成品放置到物料盘上
    node.param("product_grasp_x", product_grasp_x, 0.011);  //  成品放置到物料盘上的x坐标偏移量(笛卡尔坐标系)
    node.param("product_grasp_y", product_grasp_y, 0.0);  //　成品放置到物料盘上的y坐标偏移量(笛卡尔坐标系)
    node.param("product_grasp_z", product_grasp_z, 0.060134663);  //　成品放置到物料盘上的z坐标偏移量(笛卡尔坐标系)

    // 成品放置到立体仓库上
    node.param("Ste_warehouse_x", Ste_warehouse_x, 0.0);  //  成品放置到立体仓库上的x坐标偏移量(笛卡尔坐标系)
    node.param("Ste_warehouse_y", Ste_warehouse_y, 0.0);  //　成品放置到立体仓库上的y坐标偏移量(笛卡尔坐标系)
    node.param("Ste_warehouse_z", Ste_warehouse_z, 0.0);  //　成品放置到立体仓库上的z坐标偏移量(笛卡尔坐标系)

    // 立体仓库AR码
    node.param("arcodeid", arcodeid, 0.0);  //　成品放置到立体仓库上的z坐标偏移量(笛卡尔坐标系)

    //  机械臂抓取物料完成标志位
    ros::Publisher grasp_pub = n.advertise<std_msgs::Int32>("GraspDone", 1000);
    //  机械臂放置物料完成标志位
    ros::Publisher place_pub = n.advertise<std_msgs::Int32>("PlaceDone", 1000);
    //  成品抓取
    ros::Publisher assemble_pub = n.advertise<std_msgs::Int32>("AssembleDone", 1000);

   //   机械臂使能
    robot.automaticErrorRecovery();
    //  打开爪子
    test_DO(1);

// /*--------------------------------------------------------*/
// /*----------------导航到立体仓库---------------------------*/
// /*-------------------------------------------------------*/
    // AGV导航到立体仓库状态检测
    do {
            AGVFlag agvflag;
            ros::Subscriber agv_flag = n.subscribe("agv_state", 1, &AGVFlag::agv_callback, &agvflag);
            ros::Rate loop_rate(10);
            sleep(1);
            while(ros::ok() and agvflag.count <=0){
                ros::spinOnce();
                loop_rate.sleep();
            }
            flag = agvflag.print_agvflag();
        } while (flag != 3);

/*--------------------------------------------------------*/
/*----------------成品放置到立体仓库---------------------------*/
/*-------------------------------------------------------*/
    //  运动到中间位置
    robot_move_to_LTCK_fixed_middle_pose();

    //  运动到识别位置
    robot_move_to_LTCK_pubsh_pose();

    //  待修改，从装配台那边查询数据
    // arcodeid = 2;

    // ARCode id=4
  	if(arcodeid == 1.0){
    //  识别二维码
        // 机械臂xy初步识别
        do {
            ARPose arpose;
            ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1, &ARPose::callbackid4, &arpose);
            ros::Rate loop_rate(10);
            //sleep(1);
            while(ros::ok() and arpose.count <=0){
                ros::spinOnce();
                loop_rate.sleep();
            }
            std::cout << "After spin2 : \n";
            sx4 = arpose.print_datax();
            sy4 = arpose.print_datay();
            sz4 = arpose.print_dataz();
            ww4 = arpose.print_dataww();
            wx4 = arpose.print_datawx();
            wy4 = arpose.print_datawy();
            wz4 = arpose.print_datawz();
            //  机械臂平移
            grasp_move_xy(sx4, sy4);
        } while (0);
    
        // 机械臂识别精度校准
        do {
            ARPose arpose;
            ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1, &ARPose::callbackid4, &arpose);
            ros::Rate loop_rate(10);
            //sleep(1);
            while(ros::ok() and arpose.count <=0){
                ros::spinOnce();
                loop_rate.sleep();
            }
            std::cout << "After spin3 : \n";
            sx5 = arpose.print_datax();
            sy5 = arpose.print_datay();
            sz5 = arpose.print_dataz();
            ww5 = arpose.print_dataww();
            wx5 = arpose.print_datawx();
            wy5 = arpose.print_datawy();
            wz5 = arpose.print_datawz();
            //  机械臂识别精度校准
            if(sy5 > 0.0616){
                sy5 = (sy5 - 0.0616);
                std::cout << sy5 <<std::endl;
            }
            else{
                sy5 = -(0.0616 - sy5);
            }

            if(sx5 > 0.0035){
                sx5 = (sx5 - 0.0035);
            }
            else{
                sx5 = -(0.0035 - sx5);
            }
            //  机械臂平移
            grasp_ar_xy(sx5, sy5);
        } while (0);
    }

    // ARCode id=7
    else if(arcodeid == 2.0){
    //  识别二维码
        // 机械臂xy初步识别
        do {
            ARPose arpose;
            ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1, &ARPose::callbackid7, &arpose);
            ros::Rate loop_rate(10);
            //sleep(1);
            while(ros::ok() and arpose.count <=0){
                ros::spinOnce();
                loop_rate.sleep();
            }
            std::cout << "After spin2 : \n";
            sx4 = arpose.print_datax();
            sy4 = arpose.print_datay();
            sz4 = arpose.print_dataz();
            ww4 = arpose.print_dataww();
            wx4 = arpose.print_datawx();
            wy4 = arpose.print_datawy();
            wz4 = arpose.print_datawz();
            //  机械臂平移
            grasp_move_xy(sx4, sy4);
        } while (0);
    
        // 机械臂识别精度校准
        do {
            ARPose arpose;
            ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1, &ARPose::callbackid7, &arpose);
            ros::Rate loop_rate(10);
            //sleep(1);
            while(ros::ok() and arpose.count <=0){
                ros::spinOnce();
                loop_rate.sleep();
            }
            std::cout << "After spin3 : \n";
            sx5 = arpose.print_datax();
            sy5 = arpose.print_datay();
            sz5 = arpose.print_dataz();
            ww5 = arpose.print_dataww();
            wx5 = arpose.print_datawx();
            wy5 = arpose.print_datawy();
            wz5 = arpose.print_datawz();
            //  机械臂识别精度校准
            if(sy5 > 0.0616){
                sy5 = (sy5 - 0.0616);
                std::cout << sy5 <<std::endl;
            }
            else{
                sy5 = -(0.0616 - sy5);
            }

            if(sx5 > 0.0035){
                sx5 = (sx5 - 0.0035);
            }
            else{
                sx5 = -(0.0035 - sx5);
            }
            //  机械臂平移
            grasp_ar_xy(sx5, sy5);
        } while (0);
    } 

    // ARCode id=6
    else if(arcodeid == 3.0){
    //  识别二维码
        // 机械臂xy初步识别
        do {
            ARPose arpose;
            ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1, &ARPose::callbackid6, &arpose);
            ros::Rate loop_rate(10);
            //sleep(1);
            while(ros::ok() and arpose.count <=0){
                ros::spinOnce();
                loop_rate.sleep();
            }
            std::cout << "After spin2 : \n";
            sx4 = arpose.print_datax();
            sy4 = arpose.print_datay();
            sz4 = arpose.print_dataz();
            ww4 = arpose.print_dataww();
            wx4 = arpose.print_datawx();
            wy4 = arpose.print_datawy();
            wz4 = arpose.print_datawz();
            //  机械臂平移
            grasp_move_xy(sx4, sy4);
        } while (0);
    
        // 机械臂识别精度校准
        do {
            ARPose arpose;
            ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1, &ARPose::callbackid6, &arpose);
            ros::Rate loop_rate(10);
            //sleep(1);
            while(ros::ok() and arpose.count <=0){
                ros::spinOnce();
                loop_rate.sleep();
            }
            std::cout << "After spin3 : \n";
            sx5 = arpose.print_datax();
            sy5 = arpose.print_datay();
            sz5 = arpose.print_dataz();
            ww5 = arpose.print_dataww();
            wx5 = arpose.print_datawx();
            wy5 = arpose.print_datawy();
            wz5 = arpose.print_datawz();
            //  机械臂识别精度校准
            if(sy5 > 0.0616){
                sy5 = (sy5 - 0.0616);
                std::cout << sy5 <<std::endl;
            }
            else{
                sy5 = -(0.0616 - sy5);
            }

            if(sx5 > 0.0035){
                sx5 = (sx5 - 0.0035);
            }
            else{
                sx5 = -(0.0035 - sx5);
            }
            //  机械臂平移
            grasp_ar_xy(sx5, sy5);
        } while (0); 	
    } 

    // ARCode id=8
    else if(arcodeid == 4.0){
        do {
            ARPose arpose;
            ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1, &ARPose::callbackid8, &arpose);
            ros::Rate loop_rate(10);
            //sleep(1);
            while(ros::ok() and arpose.count <=0){
                ros::spinOnce();
                loop_rate.sleep();
            }
            std::cout << "After spin2 : \n";
            sx4 = arpose.print_datax();
            sy4 = arpose.print_datay();
            sz4 = arpose.print_dataz();
            ww4 = arpose.print_dataww();
            wx4 = arpose.print_datawx();
            wy4 = arpose.print_datawy();
            wz4 = arpose.print_datawz();
            //  机械臂平移
            grasp_move_xy(sx4, sy4);
        } while (0);
    
        // 机械臂识别精度校准
        do {
            ARPose arpose;
            ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1, &ARPose::callbackid8, &arpose);
            ros::Rate loop_rate(10);
            //sleep(1);
            while(ros::ok() and arpose.count <=0){
                ros::spinOnce();
                loop_rate.sleep();
            }
            std::cout << "After spin3 : \n";
            sx5 = arpose.print_datax();
            sy5 = arpose.print_datay();
            sz5 = arpose.print_dataz();
            ww5 = arpose.print_dataww();
            wx5 = arpose.print_datawx();
            wy5 = arpose.print_datawy();
            wz5 = arpose.print_datawz();
            //  机械臂识别精度校准
            if(sy5 > 0.0616){
                sy5 = (sy5 - 0.0616);
                std::cout << sy5 <<std::endl;
            }
            else{
                sy5 = -(0.0616 - sy5);
            }

            if(sx5 > 0.0035){
                sx5 = (sx5 - 0.0035);
            }
            else{
                sx5 = -(0.0035 - sx5);
            }
            //  机械臂平移
            grasp_ar_xy(sx5, sy5);
        } while (0);
    }         
    else{
  		arcodeid = 0;	
    } 

    // 抓取成品
    //robot_move_to_pubsh_pose();
    robot_move_to_grasp_id1_pubsh();   

    //  下去物料盘
    product_grasp_actionid(product_grasp_x, product_grasp_y, product_grasp_z);

    //  物料盘上方
    robot_move_to_place_fixed_middle_pose();

    //  运动到识别位置
    robot_move_to_LTCK_pubsh_pose();

    //  相机正对AR码
    grasp_move_xy(sx4, sy4);

    //  机械臂平移
    grasp_ar_xy(sx5, sy5);

    //  防止与立体仓库碰撞，机械臂往内运动
    do {
        //  往-y轴运动
        cart_pos current_pose, end_pose;
        //  往前运动
        robot_state = robot.receiveRobotState();
        current_pose.pos = robot_state.toolTobase_pos_m;
        end_pose.pos  = robot_state.toolTobase_pos_m;
        end_pose.pos[7] += -0.03;
        MOVEL(0.3, current_pose, end_pose, robot);
        } while (0);

    //  机械臂往-z轴运动
    do {
        cart_pos current_pose, end_pose;
        robot_state = robot.receiveRobotState();
        current_pose.pos = robot_state.toolTobase_pos_m;
        end_pose.pos  = robot_state.toolTobase_pos_m;
        end_pose.pos[11] += -sz5 + 0.25;
        MOVEL(0.3, current_pose, end_pose, robot);
        } while (0);


    //  转动90°
    robot_roation_move(-PI/2);

    //  机械臂校准位置
    do {
        cart_pos current_pose, end_pose;
        //  往前运动
        robot_state = robot.receiveRobotState();
        current_pose.pos = robot_state.toolTobase_pos_m;
        end_pose.pos  = robot_state.toolTobase_pos_m;
        end_pose.pos[3] += 0.06 + Ste_warehouse_x;
        end_pose.pos[7] += 0.11 + Ste_warehouse_y;
        end_pose.pos[11] += -0.02 + Ste_warehouse_z;
        MOVEL(0.3, current_pose, end_pose, robot);
        } while (0);

    //  放置物料
    test_DO(1); //  手爪打开

    //  机械臂往上运动
    do {
        cart_pos current_pose, end_pose;
        //  往前运动
        robot_state = robot.receiveRobotState();
        current_pose.pos = robot_state.toolTobase_pos_m;
        end_pose.pos  = robot_state.toolTobase_pos_m;
        end_pose.pos[11] += 0.04;
        MOVEL(0.3, current_pose, end_pose, robot);
        } while (0);

    // 回到起始位置
    robot_move_to_pubsh_pose();

    ros::spin();
    return 0;

}
