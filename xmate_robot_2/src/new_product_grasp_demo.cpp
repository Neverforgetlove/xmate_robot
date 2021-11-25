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
    
    //未接收到数据次数
    int no_data_time = 0;
    bool move_left = false;
    bool move_right = false;
    
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

    double robot_angle = 0.0;   // 校准角度

    //定义PI和固定位姿
    const double PI = 3.14159;

    // //机械臂识别位姿
    // std::array<double, 7> Robot_Interface.place_identify_pose = {{(-95.365406 * PI / 180), (-14.531547* PI / 180), (5.96469726 * PI / 180), (-69.987455 * PI / 180), (-2.7401447 * PI / 180), (-94.285182 * PI / 180), (-4.0333042 * PI / 180)}};
    // // 机械臂放置到物料盘与装配台中转位置
    // std::array<double, 7> Robot_Interface.place_fixed_middle_pose = {{(-30.278141 * PI / 180), (-14.834426* PI / 180), (6.0488525 * PI / 180), (-74.248970 * PI / 180), (-2.8477249 * PI / 180), (-90.563409 * PI / 180), (-2.7336559 * PI / 180)}};

    // std::array<double, 7> Robot_Interface.grasp_identify_pose = {{(-95.365406 * PI / 180), (-14.531547* PI / 180), (5.96469726 * PI / 180), (-69.987455 * PI / 180), (-2.7401447 * PI / 180), (-94.285182 * PI / 180), (-4.0333042 * PI / 180)}};

    // // 机械臂放置到物料盘与物料台中转位置
    // std::array<double, 7> Robot_Interface.grasp_fixed_middle_pose = {{(-30.278141 * PI / 180), (-14.834426* PI / 180), (6.0488525 * PI / 180), (-74.248970 * PI / 180), (-2.8477249 * PI / 180), (-90.563409 * PI / 180), (-2.7336559 * PI / 180)}};

    // // 物料放置位置
    // std::array<double,7> Robot_Interface.grasp_id1_pubsh = {{(-18.750915 * PI / 180), (-5.1412582 * PI / 180), (3.69308166 * PI / 180), (-101.94888 * PI / 180), (-1.8495998 * PI / 180), (-72.657669 * PI / 180), (-15.904083 * PI / 180)}};
    // std::array<double,7> Robot_Interface.pubsh_pose = {{(-18.750915 * PI / 180), (-5.1412582 * PI / 180), (3.69308166 * PI / 180), (-101.94888 * PI / 180), (-1.8495998 * PI / 180), (-72.657669 * PI / 180), (-15.904083 * PI / 180)}};

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

    //  抓取物料偏移量
    double grasp_x, grasp_y, grasp_z;
    //  放置物料偏移量
    double place_x, place_y, place_z;

    //钣金件放置抓取参数
    double cp_bjj_place_x,cp_bjj_place_y,cp_bjj_place_z;
    double cp_bjj_grasp_x,cp_bjj_grasp_y,cp_bjj_grasp_z;

    int LS_ID = 2;
    int LM_ID = 5;
    int BJJ_ID = 2;

    node.param("LS_ID", LS_ID, 2);
    node.param("LM_ID", LM_ID, 5);
    node.param("BJJ_ID", BJJ_ID, 2);

    // 从物料盘上面抓取钣金件参数
    node.param("cp_bjj_grasp_x", cp_bjj_grasp_x, 0.015);  //  从物料盘抓取id1物料时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("cp_bjj_grasp_y", cp_bjj_grasp_y, 0.004);  //　从物料盘抓取id1物料时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("cp_bjj_grasp_z", cp_bjj_grasp_z, 0.040134663);  //　从物料盘抓取id1物料时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小

    // 设置放置钣金件参数
    node.param("cp_bjj_place_x", cp_bjj_place_x, 0.007);  // 放置物料到装配台时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左(默认：0.007)
    node.param("cp_bjj_place_y", cp_bjj_place_y, 0.053);  //　放置物料到装配台时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前(默认：0.053)
    node.param("cp_bjj_place_z", cp_bjj_place_z, 0.253);    //　放置物料到装配台时机械臂的z坐标偏移量(笛卡尔坐标系)，往下增大(默认：0.253)



    std_msgs::Int32 tag;

    ros::Rate loop_rate(10);

	ros::Subscriber ZPT_Flag = n.subscribe("AssStand_state", 1, ZPT_CallBack);
    //  成品抓取
    ros::Publisher assemble_pub = n.advertise<std_msgs::Int32>("AssembleDone", 1000);
    Robot_Interface.Stop_Moment_Thread = true;
    //等待AGV到位信号
	while(ZPT_Current_State != 1)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    /*成品抓取*/
    try{
        Robot_Interface.Robot_Set_Speed(move_sped,robot);
        //抓取中间位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.place_fixed_middle_pose,robot);
        //抓取识别位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.place_identify_pose,robot);

        /*-----校准角度-----*/
	    while(Robot_Interface.Ur_Pose[0][0]==0.0){
            no_data_time +=1;
            if(no_data_time>5 && !move_left){
                Robot_Interface.Robot_MoveL(0.05,0.0,0.0,robot);
                move_left = true;
                no_data_time = 0;
            }else if(no_data_time > 5 && !move_right){
                Robot_Interface.Robot_MoveJ(Robot_Interface.place_identify_pose,robot);
                Robot_Interface.Robot_MoveL(-0.05,0.0,0.0,robot);
                move_right = true;
                no_data_time = 0;
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
        sx1 = Robot_Interface.Ur_Pose[0][0];
        sy1 = Robot_Interface.Ur_Pose[0][1];
        sz1 = Robot_Interface.Ur_Pose[0][2];
        ww1 = Robot_Interface.Ur_Pose[0][3];
        wx1 = Robot_Interface.Ur_Pose[0][4];
        wy1 = Robot_Interface.Ur_Pose[0][5];
        wz1 = Robot_Interface.Ur_Pose[0][6];
        
        std::cout << "初步识别ar坐标：" <<std::endl;
        std::cout << "datax:" <<sx1<<std::endl;
        std::cout << "datay:" <<sy1<<std::endl;
        std::cout << "dataz:" <<sz1<<std::endl;


        //构造四元数
        Eigen::Quaterniond quaternion2(ww1,wx1,wy1,wz1);
        //四元数转欧拉角
        Eigen::Vector3d eulerAngle2=quaternion2.matrix().eulerAngles(2,1,0);
        //弧度转角度
        robot_angle = (eulerAngle2(0) * 180) / PI;
        if(robot_angle > 90.0){
	            robot_angle = 180.0 - robot_angle;	
	    }
        if(robot_angle > 90){
            //  机械臂末端旋转
            Robot_Interface.Robot_MoveR(-PI/2 + eulerAngle2(0), robot);
        }else{
            //  机械臂末端旋转
            Robot_Interface.Robot_MoveR(eulerAngle2(0) - PI/2, robot);
        }

        std::cout<<"AR码角度： "<< robot_angle <<std::endl;
        sleep(1.0);
	    Robot_Interface.clean_ur_data();
        
	    /*-----初步校准-----*/
        //等待二维码数据
	    while(Robot_Interface.Ur_Pose[0][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
	
        sx1 = Robot_Interface.Ur_Pose[0][0];
        sy1 = Robot_Interface.Ur_Pose[0][1];
        sz1 = Robot_Interface.Ur_Pose[0][2];
        ww1 = Robot_Interface.Ur_Pose[0][3];
        wx1 = Robot_Interface.Ur_Pose[0][4];
        wy1 = Robot_Interface.Ur_Pose[0][5];
        wz1 = Robot_Interface.Ur_Pose[0][6];
        //Robot_Interface.clean_ur_data();
        std::cout << "转动后ar坐标：" <<std::endl;
        std::cout << "datax:" <<sx1<<std::endl;
        std::cout << "datay:" <<sy1<<std::endl;
        std::cout << "dataz:" <<sz1<<std::endl;

        Robot_Interface.Robot_MoveL(sx1-0.024,-sy1+0.02,0.0,robot);
        sleep(1.0);
        Robot_Interface.clean_ur_data();

        /*-----二次校准-----*/
        //等待二维码数据
	    while(Robot_Interface.Ur_Pose[0][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
	
        sx2 = Robot_Interface.Ur_Pose[0][0];
        sy2 = Robot_Interface.Ur_Pose[0][1];
        sz2 = Robot_Interface.Ur_Pose[0][2];
        ww2 = Robot_Interface.Ur_Pose[0][3];
        wx2 = Robot_Interface.Ur_Pose[0][4];
        wy2 = Robot_Interface.Ur_Pose[0][5];
        wz2 = Robot_Interface.Ur_Pose[0][6];
        //Robot_Interface.clean_ur_data();
        std::cout << "初步平移校准后ar坐标" <<std::endl;
        std::cout << "datax:" <<sx2<<std::endl;
        std::cout << "datay:" <<sy2<<std::endl;
        std::cout << "dataz:" <<sz2<<std::endl;
        
        //  机械臂识别精度校准
        if(sy2 > 0.0616){
            sy2 = (sy2 - 0.0616);
            std::cout << sy2 <<std::endl;
        }
        else{
            sy2 = -(0.0616 - sy2);
        }
        if(sx2 > 0.0035){
            sx2 = (sx2 - 0.0035);
        }
        else{
            sx2 = -(0.0035 - sx2);
        }
        Robot_Interface.Robot_MoveL(sx2,-sy2,0.0,robot);
        std::cout << "精度计算的坐标平移量" <<std::endl;
        std::cout << "datax:" <<sx2<<std::endl;
        std::cout << "datay:" <<sy2<<std::endl;
        std::cout << "dataz:" <<sz2<<std::endl;
        sleep(1.0);
        Robot_Interface.clean_ur_data();
        
        /*-----往下-----*/
        while(Robot_Interface.Ur_Pose[0][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
        sx3 = Robot_Interface.Ur_Pose[0][0];
        sy3 = Robot_Interface.Ur_Pose[0][1];
        sz3 = Robot_Interface.Ur_Pose[0][2];
        ww3 = Robot_Interface.Ur_Pose[0][3];
        wx3 = Robot_Interface.Ur_Pose[0][4];
        wy3 = Robot_Interface.Ur_Pose[0][5];
        wz3 = Robot_Interface.Ur_Pose[0][6];
        //Robot_Interface.clean_ur_data();
        std::cout << "抓取前ar坐标" <<std::endl;
        std::cout << "datax:" <<sx3<<std::endl;
        std::cout << "datay:" <<sy3<<std::endl;
        std::cout << "dataz:" <<sz3<<std::endl;

        // Robot_Interface.Robot_MoveL(sx3-cp_bjj_place_x,-sy3+cp_bjj_place_y,-(sz3-cp_bjj_place_z),robot);
        Robot_Interface.Robot_MoveL(sx3-cp_bjj_place_x,-sy3+cp_bjj_place_y,0.0,robot);
	    Robot_Interface.Robot_MoveL(0.0,0.0,-(sz3-cp_bjj_place_z),robot);
        // Robot_Interface.Robot_MoveL(0.0,0.0,sz3-cp_bjj_place_z,robot);
        bool close_state2 = Robot_Interface.Robot_Grasp_Control(0,robot);
        if(close_state2){
            //往上
            Robot_Interface.Robot_MoveL(0.0, 0.0, sz3-cp_bjj_place_z, robot);
            //物料盘上方
            Robot_Interface.Robot_MoveJ(Robot_Interface.place_identify_pose,robot);
            //放置中间位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.place_fixed_middle_pose,robot);
            //放置识别位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
        }
        sleep(1.0);
        Robot_Interface.clean_ur_data();
	    Robot_Interface.Robot_MoveL(-cp_bjj_grasp_x, cp_bjj_grasp_y, 0.0, robot);
        Robot_Interface.Robot_MoveL(0.0, 0.0, -cp_bjj_grasp_z, robot);
        //开夹爪
	    bool open_state2 = Robot_Interface.Robot_Grasp_Control(1,robot);
        if(open_state2){
            //往上
            Robot_Interface.Robot_MoveL(0.0, 0.0, cp_bjj_grasp_z, robot);
            //物料盘上方
            Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
        }
        tag.data = 1;
        assemble_pub.publish(tag);
        tag.data = 0;

    }catch (xmate::ControlException &e) {
        std::cout << e.what() << std::endl;
    }
}
