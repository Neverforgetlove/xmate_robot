#include "ros/ros.h"
#include "hg_ai_robot.h"
#include "hg_ai_robot.cpp"
//宏定义max min函数
#define max(a,b) ((a) > (b) ? (a) : (b))
#define min(a,b) ((a) < (b) ? (a) : (b))

//AGV当前目标点
int AGV_Current_Goal = 0;
//装配台状态
int ZPT_Current_State = 0;
//机械臂状态
int Robot_Current_State = 0;

//AGV回调
void AGV_CallBack(const std_msgs::Int32::ConstPtr& msg)
{
    AGV_Current_Goal = msg->data;
}
//装配台回调
void ZPT_CallBack(const std_msgs::Int32::ConstPtr& msg)
{
    ZPT_Current_State = msg->data;
}
//机械臂回调
void Robot_CallBack(const std_msgs::Int32::ConstPtr& msg)
{
    Robot_Current_State = msg->data;
}

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "ros_core");
    ros::NodeHandle n; 
    ros::NodeHandle node("~");

    double move_sped = 0.2;
    bool start_moment = false;
    //机械臂移动速度
    node.param("move_sped", move_sped, 0.2);
    //启动力矩监听
    node.param("monitor_state", start_moment, false);
    
    //初始化
    HG_AI_Robot Robot_Interface;
    Robot_Interface.Jog0_Robot_Moment = 3;
    Robot_Interface.Jog4_Robot_Moment = 25;
    Robot_Interface.Start_Moment_Thread = start_moment;

    //连接机械臂: ip地址　端口号
    xmate::Robot robot(Robot_Interface.ipaddr, Robot_Interface.port);
    sleep(1.0);
    //初始化，包括上电，初始化位姿，关节角，夹爪打开
    Robot_Interface.Init(robot);
    //获取上电状态
    int power_state = Robot_Interface.Robot_Get_Power(robot);
    //监听ar码
    ros::Subscriber marker_sub = n.subscribe("ar_pose_marker", 1, &HG_AI_Robot::Get_AR_Pose, &Robot_Interface);
    //判断上电状态，未上电则上电Start_Moment_Thread
    if(!power_state)
    {
        Robot_Interface.Robot_Set_Power(1,robot);
    }

    //定义PI和固定位姿
    const double PI = 3.14159;
    std::array<double, 7> q_init;

    // std::array<double,7> Robot_Interface.LTCK_pubsh_pose = {{(-91.486958 * PI / 180), (-25.924667 * PI / 180), (0.56589202 * PI / 180), (-72.387583 * PI / 180), (-1.7405090 * PI / 180), (-81.707141 * PI / 180), (3.72719764 * PI / 180)}};
    // std::array<double, 7> Robot_Interface.LTCK_fixed_middle_pose = {{(-18.833958 * PI / 180), (-15.501869* PI / 180), (0.74440612 * PI / 180), (-84.170942 * PI / 180), (-0.5817432 * PI / 180), (-80.469978 * PI / 180), (-9.4580955 * PI / 180)}};
    // std::array<double,7> Robot_Interface.pubsh_pose = {{(-18.750915 * PI / 180), (-5.1412582 * PI / 180), (3.69308166 * PI / 180), (-101.94888 * PI / 180), (-1.8495998 * PI / 180), (-72.657669 * PI / 180), (-15.904083 * PI / 180)}};

    //速度限制
    move_sped = min(move_sped, 0.3);

    ros::Rate loop_rate(10);

    // Robot_Interface.Stop_Moment_Thread = true;

/*--------------------id1,螺母--------------------*/

    try{
	    Robot_Interface.Robot_Set_Speed(move_sped,robot);
        //抓取中间位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_fixed_middle_pose,robot);
        //抓取识别位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_pubsh_pose,robot);
        
        Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_fixed_middle_pose,robot);
            
        Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);

    }catch (xmate::ControlException &e) {
        std::cout << e.what() << std::endl;
    }
    ros::spin();
    return 0;
}
