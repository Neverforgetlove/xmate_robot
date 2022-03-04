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
    node.param("monitor_state", start_moment, true);
    
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
    ros::Subscriber ur_marker_sub = n.subscribe("/aruco_single/pose", 1, &HG_AI_Robot::Get_UR_Pose, &Robot_Interface);

    //判断上电状态，未上电则上电Start_Moment_Thread
    if(!power_state)
    {
        Robot_Interface.Robot_Set_Power(1,robot);
    }
    //初始化参数
 	float sx ,sy ,sz ,ww ,wx ,wy, wz;
	float sx1 ,sy1 ,sz1 ,ww1 ,wx1 ,wy1, wz1;

    double robot_angle = 0.0;   // 校准角度

    //定义PI和固定位姿
    const double PI = 3.14159;

    // std::array<double,7> Robot_Interface.LTCK_Robot_Interface.pubsh_pose = {{(-91.486958 * PI / 180), (-25.924667 * PI / 180), (0.56589202 * PI / 180), (-72.387583 * PI / 180), (-1.7405090 * PI / 180), (-81.707141 * PI / 180), (3.72719764 * PI / 180)}};
    // std::array<double, 7> Robot_Interface.LTCK_fixed_middle_pose = {{(-18.833958 * PI / 180), (-15.501869* PI / 180), (0.74440612 * PI / 180), (-84.170942 * PI / 180), (-0.5817432 * PI / 180), (-80.469978 * PI / 180), (-9.4580955 * PI / 180)}};
    
    // // 物料放置位置
    // std::array<double,7> Robot_Interface.grasp_id1_pubsh = {{(-18.750915 * PI / 180), (-5.1412582 * PI / 180), (3.69308166 * PI / 180), (-101.94888 * PI / 180), (-1.8495998 * PI / 180), (-72.657669 * PI / 180), (-15.904083 * PI / 180)}};
    // std::array<double,7> Robot_Interface.pubsh_pose = {{(-18.750915 * PI / 180), (-5.1412582 * PI / 180), (3.69308166 * PI / 180), (-101.94888 * PI / 180), (-1.8495998 * PI / 180), (-72.657669 * PI / 180), (-15.904083 * PI / 180)}};

    int arcodeid = 0;
    int push_id = 0;
    int ID_Name[4] = {4,6,7,8};    

    //  将成品放置到物料盘上
    double product_x, product_y, product_z;
    //  从物料盘上抓取成品
    double product_grasp_x, product_grasp_y, product_grasp_z; 

    //  将成品放置到立体仓库
    double Ste_warehouse_x, Ste_warehouse_y, Ste_warehouse_z; 

    node.param("arcodeid", arcodeid, 2);

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

    //速度限制
    move_sped = min(move_sped, 0.3);

    std_msgs::Int32 tag;

    ros::Rate loop_rate(10);

	ros::Subscriber agv_flag = n.subscribe("agv_state", 1, AGV_CallBack);
    Robot_Interface.Stop_Moment_Thread = true;
    push_id = arcodeid;
    // 如果放0,1号位开启碰撞检测
    if(arcodeid == 0 || arcodeid == 1)
    {
        Robot_Interface.Start_Moment_Thread = true;
        Robot_Interface.Stop_Moment_Thread = false;
    }

    arcodeid = ID_Name[arcodeid];
    //等待AGV到位信号
	while(AGV_Current_Goal != 3)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    /*成品抓取*/
    try{
        Robot_Interface.Robot_Set_Speed(move_sped,robot);
        if(push_id == 0 || push_id == 2)
        {
            //旋转末端
            Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_left_middle_pose,robot);
            //抓取中间位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_left_middle_push_pose,robot);
            //抓取识别位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_left_push_pose,robot);
        }else{
            //抓取中间位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_fixed_middle_pose,robot);
            //抓取识别位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_pubsh_pose,robot);
        }
        /*-----校准角度-----*/
	    while(Robot_Interface.Ar_Pose[arcodeid][0]==0.0){
            ros::spinOnce();
            loop_rate.sleep();
        }
        sx = Robot_Interface.Ar_Pose[arcodeid][0];
        sy = Robot_Interface.Ar_Pose[arcodeid][1];
        sz = Robot_Interface.Ar_Pose[arcodeid][2];
        ww = Robot_Interface.Ar_Pose[arcodeid][3];
        wx = Robot_Interface.Ar_Pose[arcodeid][4];
        wy = Robot_Interface.Ar_Pose[arcodeid][5];
        wz = Robot_Interface.Ar_Pose[arcodeid][6];
        
        Robot_Interface.Robot_MoveL(sx-0.05,-sy+0.02,0.0,robot);

        std::cout << "初步识别ar坐标：" <<std::endl;
        std::cout << "datax:" <<sx<<std::endl;
        std::cout << "datay:" <<sy<<std::endl;
        std::cout << "dataz:" <<sz<<std::endl;
        sleep(1.0);
        Robot_Interface.clean_ar_data();

        while(Robot_Interface.Ar_Pose[arcodeid][0]==0.0){
            ros::spinOnce();
            loop_rate.sleep();
        }
        sx1 = Robot_Interface.Ar_Pose[arcodeid][0];
        sy1 = Robot_Interface.Ar_Pose[arcodeid][1];
        sz1 = Robot_Interface.Ar_Pose[arcodeid][2];
        ww1 = Robot_Interface.Ar_Pose[arcodeid][3];
        wx1 = Robot_Interface.Ar_Pose[arcodeid][4];
        wy1 = Robot_Interface.Ar_Pose[arcodeid][5];
        wz1 = Robot_Interface.Ar_Pose[arcodeid][6];

        if(sy1 > 0.0616){
                sy1 = (sy1 - 0.0616);
                std::cout << sy1 <<std::endl;
            }
        else{
            sy1 = -(0.0616 - sy1);
        }

        if(sx1 > 0.0035){
            sx1 = (sx1 - 0.0035);
        }
        else{
            sx1 = -(0.0035 - sx1);
        }

        Robot_Interface.Robot_MoveL(sx1,-sy1,0.0,robot);

        std::cout << "二次识别ar坐标：" <<std::endl;
        std::cout << "datax:" <<sx1<<std::endl;
        std::cout << "datay:" <<sy1<<std::endl;
        std::cout << "dataz:" <<sz1<<std::endl;
        sleep(1.0);
        Robot_Interface.clean_ar_data();

        // 放左边先旋转缩回
        if(push_id == 0 || push_id == 1){
            Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_left_middle_push_pose,robot);
            Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_left_middle_pose,robot);
        }
        
        Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_id1_pubsh,robot);
        Robot_Interface.Robot_MoveL(-product_grasp_x,product_grasp_y,0.0,robot);

        Robot_Interface.Robot_MoveL(0.0,0.0,-product_grasp_z,robot);
        bool close_state1 = Robot_Interface.Robot_Grasp_Control(0,robot);
        if(close_state1){
            Robot_Interface.Robot_MoveL(0.0,0.0,product_grasp_z,robot);
            
            if(push_id == 0 || push_id == 1){
                //旋转末端
                Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_left_middle_pose,robot);
                //抓取中间位姿
                Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_left_middle_push_pose,robot);
                //抓取识别位姿
                Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_left_push_pose,robot);
            }else{
                //抓取中间位姿
                Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_fixed_middle_pose,robot);
                //抓取识别位姿
                Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_pubsh_pose,robot);
            }
        }
        // Robot_Interface.Robot_MoveL(sx-0.05,-sy+0.02,0.0,robot);
        // Robot_Interface.Robot_MoveL(sx1,-sy1,0.0,robot);
        // Robot_Interface.Robot_MoveL(0.0,-0.03,0.0,robot);
        // Robot_Interface.Robot_MoveL(0.0,0.0,-sz1+0.25,robot);
        // Robot_Interface.Robot_MoveR(-PI/2, robot);
        Robot_Interface.Robot_MoveL(sx-0.05,-sy+0.02,0.0,robot);
        Robot_Interface.Robot_MoveL(sx1,-sy1,0.0,robot);
        Robot_Interface.Robot_MoveL(Ste_warehouse_x, Ste_warehouse_y, 0.0, robot);
        Robot_Interface.Robot_MoveL(0.0, 0.0, Ste_warehouse_z,robot);
        bool open_state = Robot_Interface.Robot_Grasp_Control(1,robot);
        if(open_state){
            Robot_Interface.Robot_MoveL(0.0,0.0,0.04,robot);
            if(push_id == 0|| push_id == 1)
            {
                Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_left_middle_push_pose,robot);
                Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_left_middle_pose,robot);
            }else{
                //放置识别位姿
                Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_fixed_middle_pose,robot);
            }
        }
        Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
    }catch (xmate::ControlException &e) {
        std::cout << e.what() << std::endl;
    }
}
