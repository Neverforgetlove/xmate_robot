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

    //未接收到数据次数
    int no_data_time = 0;
    bool move_left = false;
    bool move_right = false;
    int grasp_obj_state[3] = {0,0,0};

    //连接机械臂: ip地址　端口号
    xmate::Robot robot(Robot_Interface.ipaddr, Robot_Interface.port);
    sleep(1.0);
    //初始化，包括上电，初始化位姿，关节角，夹爪打开
    Robot_Interface.Init(robot);
    //获取上电状态
    int power_state = Robot_Interface.Robot_Get_Power(robot);
    //监听ar码
    ros::Subscriber marker_sub = n.subscribe("ar_pose_marker", 1, &HG_AI_Robot::Get_AR_Pose, &Robot_Interface);
    //监听ur码
    ros::Subscriber aruco_marker_sub = n.subscribe("/aruco_single/pose", 1, &HG_AI_Robot::Get_UR_Pose, &Robot_Interface);

    //判断上电状态，未上电则上电Start_Moment_Thread
    if(!power_state)
    {
        Robot_Interface.Robot_Set_Power(1,robot);
    }

    //初始化参数
 	float sx ,sy ,sz ,ww ,wx ,wy, wz;
	float sx1 ,sy1 ,sz1 ,ww1 ,wx1 ,wy1, wz1;
     //  机械臂平移运动校准
    float sx2 ,sy2 ,sz2 ,ww2 ,wx2 ,wy2, wz2;
     //  机械臂平移运动放置
    float sx3 ,sy3 ,sz3 ,ww3 ,wx3 ,wy3, wz3;

    //放置完成信号
    std_msgs::Int32 tag;

    //抓取物料类型设置
    int grasp_type = 1;
    //抓取类型对应的二维码号, 螺丝螺母钣金件
    int grasp_ar_id[3] = {0,1,2}; 
    //放置顺序对应的AR码ID
    int LS_ID = 0;
    int LM_ID = 1;
    int BJJ_ID = 2;

    //货仓位置
    int arcodeid = 0;
    int ID_Name[4] = {4,6,7,8};   

    //  抓取物料偏移量
	double grasp_x, grasp_y, grasp_z;
    //  放置物料偏移量
    double place_x, place_y, place_z;

    //  将id1物料放置到物料盘上偏移量
	double graspid1_x, graspid1_y, graspid1_z;
    //  将id2物料放置到物料盘上偏移量
	double graspid2_x, graspid2_y, graspid2_z;
    //  将id1物料放置到装配台上偏移量
    double placeid1_x, placeid1_y, placeid1_z;
    //  将id2物料放置到装配台上偏移量
    double placeid2_x, placeid2_y, placeid2_z;


    //钣金件抓取放置偏移量
    double bjj_grasp_x, bjj_grasp_y, bjj_grasp_z;
    double bjj_place_x,bjj_place_y,bjj_place_z;
    
    //装配台钣金件抓取放置偏移量
    double zpt_bjj_grasp_x, zpt_bjj_grasp_y, zpt_bjj_grasp_z;
    double zpt_bjj_place_x,zpt_bjj_place_y,zpt_bjj_place_z;

    //成品抓取放置偏移量
    double cp_bjj_grasp_x, cp_bjj_grasp_y, cp_bjj_grasp_z;
    double cp_bjj_place_x, cp_bjj_place_y, cp_bjj_place_z;

    //  从物料盘上抓取成品
    double product_grasp_x, product_grasp_y, product_grasp_z; 
    //  将成品放置到立体仓库
    double Ste_warehouse_x, Ste_warehouse_y, Ste_warehouse_z; 

    double robot_angle = 0.0;   // 校准角度

    //定义PI和固定位姿
    const double PI = 3.14159;
    std::array<double, 7> q_init;

    // // 机械臂抓取识别位姿
    // std::array<double, 7> Robot_Interface.grasp_identify_pose = {{(-95.365406 * PI / 180), (-14.531547* PI / 180), (5.96469726 * PI / 180), (-69.987455 * PI / 180), (-2.7401447 * PI / 180), (-94.285182 * PI / 180), (-4.0333042 * PI / 180)}};

    // // 机械臂放置到物料盘与物料台中转位置
    // std::array<double, 7> Robot_Interface.grasp_fixed_middle_pose = {{(-30.278141 * PI / 180), (-14.834426* PI / 180), (6.0488525 * PI / 180), (-74.248970 * PI / 180), (-2.8477249 * PI / 180), (-90.563409 * PI / 180), (-2.7336559 * PI / 180)}};

    // // 物料放置位置
    // std::array<double,7> Robot_Interface.grasp_id1_pubsh = {{(-18.750915 * PI / 180), (-5.1412582 * PI / 180), (3.69308166 * PI / 180), (-101.94888 * PI / 180), (-1.8495998 * PI / 180), (-72.657669 * PI / 180), (-15.904083 * PI / 180)}};
    // std::array<double,7> Robot_Interface.pubsh_pose = {{(-18.750915 * PI / 180), (-5.1412582 * PI / 180), (3.69308166 * PI / 180), (-101.94888 * PI / 180), (-1.8495998 * PI / 180), (-72.657669 * PI / 180), (-15.904083 * PI / 180)}};
    
    // //机械臂识别位姿
    // std::array<double, 7> Robot_Interface.place_identify_pose = {{(-95.365406 * PI / 180), (-14.531547* PI / 180), (5.96469726 * PI / 180), (-69.987455 * PI / 180), (-2.7401447 * PI / 180), (-94.285182 * PI / 180), (-4.0333042 * PI / 180)}};
    // // 机械臂放置到物料盘与装配台中转位置
    // std::array<double, 7> Robot_Interface.place_fixed_middle_pose = {{(-30.278141 * PI / 180), (-14.834426* PI / 180), (6.0488525 * PI / 180), (-74.248970 * PI / 180), (-2.8477249 * PI / 180), (-90.563409 * PI / 180), (-2.7336559 * PI / 180)}};

    // //立体仓库
    // std::array<double,7> Robot_Interface.LTCK_pubsh_pose = {{(-91.486958 * PI / 180), (-25.924667 * PI / 180), (0.56589202 * PI / 180), (-72.387583 * PI / 180), (-1.7405090 * PI / 180), (-81.707141 * PI / 180), (3.72719764 * PI / 180)}};
    // std::array<double, 7> Robot_Interface.LTCK_fixed_middle_pose = {{(-18.833958 * PI / 180), (-15.501869* PI / 180), (0.74440612 * PI / 180), (-84.170942 * PI / 180), (-0.5817432 * PI / 180), (-80.469978 * PI / 180), (-9.4580955 * PI / 180)}};

    //机械臂抓取类型
    node.param("target_type", grasp_type, 1);

    //放置顺序对应的AR码ID
    node.param("LS_ID", LS_ID, 0);
    node.param("LM_ID", LM_ID, 1);
    node.param("BJJ_ID", BJJ_ID, 2);

    node.param("arcodeid", arcodeid, 2);

    // 设置抓取物料参数
    node.param("grasp_x", grasp_x, 0.020);  // 抓取物料时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("grasp_y", grasp_y, 0.053);  //　抓取物料时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("grasp_z", grasp_z, 0.195);    //　抓取物料时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小


    // 物料id1在物料盘上面放置参数
    node.param("graspid1_x", graspid1_x, 0.026);  //  抓取id1物料到物料盘时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("graspid1_y", graspid1_y, -0.053);  //　抓取id1物料到物料盘时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("graspid1_z", graspid1_z, 0.056);  //　抓取id1物料到物料盘时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小

    // 物料id2在物料盘上面放置参数
    node.param("graspid2_x", graspid2_x, 0.026);  //  抓取id1物料到物料盘时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("graspid2_y", graspid2_y, 0.068);  //　抓取id1物料到物料盘时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("graspid2_z", graspid2_z, 0.056);  //　抓取id1物料到物料盘时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小
    
    // 从物料盘上面抓取钣金件参数
    node.param("bjj_grasp_x", bjj_grasp_x, 0.015);  //  从物料盘抓取id1物料时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("bjj_grasp_y", bjj_grasp_y, 0.004);  //　从物料盘抓取id1物料时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("bjj_grasp_z", bjj_grasp_z, 0.040134663);  //　从物料盘抓取id1物料时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小

    // 设置放置钣金件参数
    node.param("bjj_place_x", bjj_place_x, 0.007);  // 放置物料到装配台时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左(默认：0.007)
    node.param("bjj_place_y", bjj_place_y, 0.053);  //　放置物料到装配台时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前(默认：0.053)
    node.param("bjj_place_z", bjj_place_z, 0.253);    //　放置物料到装配台时机械臂的z坐标偏移量(笛卡尔坐标系)，往下增大(默认：0.253)

    //放置顺序对应的AR码ID
    node.param("LS_ID", LS_ID, 0);
    node.param("LM_ID", LM_ID, 1);
    node.param("BJJ_ID", BJJ_ID, 2);

    // 设置放置物料参数
    node.param("place_x", place_x, 0.007);  // 放置物料到装配台时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左(默认：0.007)
    node.param("place_y", place_y, 0.053);  //　放置物料到装配台时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前(默认：0.053)
    node.param("place_z", place_z, 0.253);    //　放置物料到装配台时机械臂的z坐标偏移量(笛卡尔坐标系)，往下增大(默认：0.253)


    // 从物料盘上面抓取钣金件参数
    node.param("zpt_bjj_grasp_x", zpt_bjj_grasp_x, 0.015);  //  从物料盘抓取id1物料时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("zpt_bjj_grasp_y", zpt_bjj_grasp_y, 0.004);  //　从物料盘抓取id1物料时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("zpt_bjj_grasp_z", zpt_bjj_grasp_z, 0.040134663);  //　从物料盘抓取id1物料时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小

    // 设置放置钣金件参数
    node.param("zpt_bjj_place_x", zpt_bjj_place_x, 0.007);  // 放置物料到装配台时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左(默认：0.007)
    node.param("zpt_bjj_place_y", zpt_bjj_place_y, 0.053);  //　放置物料到装配台时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前(默认：0.053)
    node.param("zpt_bjj_place_z", zpt_bjj_place_z, 0.253);    //　放置物料到装配台时机械臂的z坐标偏移量(笛卡尔坐标系)，往下增大(默认：0.253)

    // 从物料盘上面抓取钣金件参数
    node.param("cp_bjj_grasp_x", cp_bjj_grasp_x, 0.015);  //  从物料盘抓取id1物料时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("cp_bjj_grasp_y", cp_bjj_grasp_y, 0.004);  //　从物料盘抓取id1物料时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("cp_bjj_grasp_z", cp_bjj_grasp_z, 0.040134663);  //　从物料盘抓取id1物料时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小

    // 设置放置钣金件参数
    node.param("cp_bjj_place_x", cp_bjj_place_x, 0.007);  // 放置物料到装配台时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左(默认：0.007)
    node.param("cp_bjj_place_y", cp_bjj_place_y, 0.053);  //　放置物料到装配台时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前(默认：0.053)
    node.param("cp_bjj_place_z", cp_bjj_place_z, 0.253);    //　放置物料到装配台时机械臂的z坐标偏移量(笛卡尔坐标系)，往下增大(默认：0.253)


    // 物料id1从物料盘上面抓取参数
    node.param("placeid1_x", placeid1_x, 0.015);  //  从物料盘抓取id1物料时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("placeid1_y", placeid1_y, 0.004);  //　从物料盘抓取id1物料时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("placeid1_z", placeid1_z, 0.040134663);  //　从物料盘抓取id1物料时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小

    // 物料id2从物料盘上面抓取参数
    node.param("placeid2_x", placeid2_x, 0.0);  //  从物料盘抓取id2物料时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("placeid2_y", placeid2_y, 0.1115);  //　从物料盘抓取id2物料时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("placeid2_z", placeid2_z, 0.041234663);  //　从物料盘抓取id2物料时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小
    
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

    if(grasp_type == 1){
        grasp_ar_id[0] = 0;
        grasp_ar_id[1] = 1;
        grasp_ar_id[2] = 2;
    }else if(grasp_type == 2){
        grasp_ar_id[0] = 3;
        grasp_ar_id[1] = 4;
        grasp_ar_id[2] = 5;
    }
    arcodeid = ID_Name[arcodeid];
    ros::Rate loop_rate(10);

    // 机械臂抓取物料完成标志位
    ros::Publisher grasp_pub = n.advertise<std_msgs::Int32>("GraspDone", 1000);
	ros::Subscriber agv_flag = n.subscribe("agv_state", 1, AGV_CallBack);
    
    //  机械臂放置物料完成标志位
    ros::Publisher place_pub = n.advertise<std_msgs::Int32>("PlaceDone", 1000);
    //  成品抓取
    ros::Publisher assemble_pub = n.advertise<std_msgs::Int32>("AssembleDone", 1000);
    ros::Subscriber assstand_flag = n.subscribe("AssStand_state", 1, ZPT_CallBack);

    Robot_Interface.Stop_Moment_Thread = true;
    //等待AGV到位信号
	while(AGV_Current_Goal != 1)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

/*--------------------id2,螺丝--------------------*/

    try{
	    Robot_Interface.Robot_Set_Speed(move_sped,robot);
        //抓取中间位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_fixed_middle_pose,robot);
        //抓取识别位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_identify_pose,robot);

        /*-----校准角度-----*/
	    while(Robot_Interface.Ar_Pose[grasp_ar_id[2]][0]==0.0){
            no_data_time +=1;
            if(no_data_time>5 && !move_left){
                Robot_Interface.Robot_MoveL(0.05,0.0,0.0,robot);
                move_left = true;
                no_data_time = 0;
            }else if(no_data_time > 5 && !move_right){
                Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_identify_pose,robot);
                Robot_Interface.Robot_MoveL(-0.05,0.0,0.0,robot);
                move_right = true;
                no_data_time = 0;
            }
            ros::spinOnce();
            loop_rate.sleep();
        }

        if (move_left && !move_right)
        {
            std::cout<<"grasp target1 move left"<<std::endl;
            move_left = move_right = false;
        }else if(move_left && move_right)
        {
            std::cout<<"grasp target1 move right"<<std::endl;
            move_left = move_right = false;
        }

        sx = Robot_Interface.Ar_Pose[grasp_ar_id[2]][0];
        sy = Robot_Interface.Ar_Pose[grasp_ar_id[2]][1];
        sz = Robot_Interface.Ar_Pose[grasp_ar_id[2]][2];
        ww = Robot_Interface.Ar_Pose[grasp_ar_id[2]][3];
        wx = Robot_Interface.Ar_Pose[grasp_ar_id[2]][4];
        wy = Robot_Interface.Ar_Pose[grasp_ar_id[2]][5];
        wz = Robot_Interface.Ar_Pose[grasp_ar_id[2]][6];
        
        std::cout << "初步识别ar坐标：" <<std::endl;
        std::cout << "datax:" <<sx<<std::endl;
        std::cout << "datay:" <<sy<<std::endl;
        std::cout << "dataz:" <<sz<<std::endl;


        //构造四元数
        Eigen::Quaterniond quaternion(ww,wx,wy,wz);
        //四元数转欧拉角
        Eigen::Vector3d eulerAngle=quaternion.matrix().eulerAngles(2,1,0);
        //弧度转角度
        robot_angle = (eulerAngle(0) * 180) / PI;
        if(robot_angle > 90.0){
	            robot_angle = 180.0 - robot_angle;	
	    }
        if(robot_angle > 90){
            //  机械臂末端旋转
            Robot_Interface.Robot_MoveR(-PI/2 + eulerAngle(0), robot);
        }else{
            //  机械臂末端旋转
            Robot_Interface.Robot_MoveR(eulerAngle(0) - PI/2, robot);
        }

        std::cout<<"AR码角度： "<< robot_angle <<std::endl;
        sleep(1.0);
	    Robot_Interface.clean_ar_data();

        /*-----初步校准-----*/
        //等待二维码数据
	    while(Robot_Interface.Ar_Pose[grasp_ar_id[2]][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
	
        sx = Robot_Interface.Ar_Pose[grasp_ar_id[2]][0];
        sy = Robot_Interface.Ar_Pose[grasp_ar_id[2]][1];
        sz = Robot_Interface.Ar_Pose[grasp_ar_id[2]][2];
        ww = Robot_Interface.Ar_Pose[grasp_ar_id[2]][3];
        wx = Robot_Interface.Ar_Pose[grasp_ar_id[2]][4];
        wy = Robot_Interface.Ar_Pose[grasp_ar_id[2]][5];
        wz = Robot_Interface.Ar_Pose[grasp_ar_id[2]][6];
        
        std::cout << "转动后ar坐标：" <<std::endl;
        std::cout << "datax:" <<sx<<std::endl;
        std::cout << "datay:" <<sy<<std::endl;
        std::cout << "dataz:" <<sz<<std::endl;

        Robot_Interface.Robot_MoveL(sx-0.05,-sy+0.02,0.0,robot);
        sleep(1.0);
        Robot_Interface.clean_ar_data();

        /*-----二次校准-----*/
        //等待二维码数据
	    while(Robot_Interface.Ar_Pose[grasp_ar_id[2]][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
	
        sx = Robot_Interface.Ar_Pose[grasp_ar_id[2]][0];
        sy = Robot_Interface.Ar_Pose[grasp_ar_id[2]][1];
        sz = Robot_Interface.Ar_Pose[grasp_ar_id[2]][2];
        ww = Robot_Interface.Ar_Pose[grasp_ar_id[2]][3];
        wx = Robot_Interface.Ar_Pose[grasp_ar_id[2]][4];
        wy = Robot_Interface.Ar_Pose[grasp_ar_id[2]][5];
        wz = Robot_Interface.Ar_Pose[grasp_ar_id[2]][6];
        
        std::cout << "初步平移校准后ar坐标" <<std::endl;
        std::cout << "datax:" <<sx<<std::endl;
        std::cout << "datay:" <<sy<<std::endl;
        std::cout << "dataz:" <<sz<<std::endl;
        
        //  机械臂识别精度校准
        if(sy > 0.0616){
            sy = (sy - 0.0616);
            std::cout << sy <<std::endl;
        }
        else{
            sy = -(0.0616 - sy);
        }
        if(sx > 0.0035){
            sx = (sx - 0.0035);
        }
        else{
            sx = -(0.0035 - sx);
        }
        Robot_Interface.Robot_MoveL(sx,-sy,0.0,robot);
        std::cout << "精度计算的坐标平移量" <<std::endl;
        std::cout << "datax:" <<sx<<std::endl;
        std::cout << "datay:" <<sy<<std::endl;
        std::cout << "dataz:" <<sz<<std::endl;
        sleep(1.0);
        Robot_Interface.clean_ar_data();
        
        /*-----下去抓-----*/
        while(Robot_Interface.Ar_Pose[grasp_ar_id[2]][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
        sx = Robot_Interface.Ar_Pose[grasp_ar_id[2]][0];
        sy = Robot_Interface.Ar_Pose[grasp_ar_id[2]][1];
        sz = Robot_Interface.Ar_Pose[grasp_ar_id[2]][2];
        ww = Robot_Interface.Ar_Pose[grasp_ar_id[2]][3];
        wx = Robot_Interface.Ar_Pose[grasp_ar_id[2]][4];
        wy = Robot_Interface.Ar_Pose[grasp_ar_id[2]][5];
        wz = Robot_Interface.Ar_Pose[grasp_ar_id[2]][6];
        
        std::cout << "抓取前ar坐标" <<std::endl;
        std::cout << "datax:" <<sx<<std::endl;
        std::cout << "datay:" <<sy<<std::endl;
        std::cout << "dataz:" <<sz<<std::endl;
        
        Robot_Interface.Robot_MoveL(sx-grasp_x,-sy+grasp_y,0.0,robot);
        Robot_Interface.Robot_MoveL(0.0,0.0,-sz+grasp_z,robot);
        sleep(1.0);
        Robot_Interface.clean_ar_data();
        //关夹爪
	    bool close_state = Robot_Interface.Robot_Grasp_Control(0,robot);
        if(close_state){
            //往上
            Robot_Interface.Robot_MoveL(0.0,0.0,graspid2_z,robot);
            Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_identify_pose,robot);
            //中间位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_fixed_middle_pose,robot);
            //初始位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
            //放置２号位姿
            Robot_Interface.Robot_MoveL(graspid2_x,graspid2_y,0.0,robot);
            //往下
            Robot_Interface.Robot_MoveL(0.0,0.0,-graspid2_z,robot);
        }
	    bool open_state = Robot_Interface.Robot_Grasp_Control(1,robot);
	    //往上
        Robot_Interface.Robot_MoveL(0.0,0.0,graspid2_z,robot);
        if(open_state){
            //初始位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
        }

/*-----------------------------id1 螺母-----------------------------*/
        //速度
        Robot_Interface.Robot_Set_Speed(move_sped,robot);
        //抓取中间位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_fixed_middle_pose,robot);
        //抓取识别位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_identify_pose,robot);

        /*-----校准角度-----*/
	    while(Robot_Interface.Ar_Pose[grasp_ar_id[1]][0]==0.0){
            no_data_time +=1;
            if(no_data_time>5 && !move_left){
                Robot_Interface.Robot_MoveL(0.05,0.0,0.0,robot);
                move_left = true;
                no_data_time = 0;
            }else if(no_data_time > 5 && !move_right){
                Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_identify_pose,robot);
                Robot_Interface.Robot_MoveL(-0.05,0.0,0.0,robot);
                move_right = true;
                no_data_time = 0;
            }
            ros::spinOnce();
            loop_rate.sleep();
        }

        if (move_left && !move_right)
        {
            std::cout<<"grasp target2 move left"<<std::endl;
            move_left = move_right = false;
        }else if(move_left && move_right)
        {
            std::cout<<"grasp target2 move right"<<std::endl;
            move_left = move_right = false;
        }

        sx1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][0];
        sy1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][1];
        sz1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][2];
        ww1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][3];
        wx1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][4];
        wy1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][5];
        wz1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][6];
        
        std::cout << "初步识别ar坐标：" <<std::endl;
        std::cout << "datax:" <<sx1<<std::endl;
        std::cout << "datay:" <<sy1<<std::endl;
        std::cout << "dataz:" <<sz1<<std::endl;


        //构造四元数
        Eigen::Quaterniond quaternion1(ww1,wx1,wy1,wz1);
        //四元数转欧拉角
        Eigen::Vector3d eulerAngle1=quaternion1.matrix().eulerAngles(2,1,0);
        //弧度转角度
        robot_angle = (eulerAngle1(0) * 180) / PI;
        if(robot_angle > 90.0){
	            robot_angle = 180.0 - robot_angle;	
	    }
        if(robot_angle > 90){
            //  机械臂末端旋转
            Robot_Interface.Robot_MoveR(-PI/2 + eulerAngle1(0), robot);
        }else{
            //  机械臂末端旋转
            Robot_Interface.Robot_MoveR(eulerAngle1(0) - PI/2, robot);
        }

        std::cout<<"AR码角度： "<< robot_angle <<std::endl;
        sleep(1.0);
	    Robot_Interface.clean_ar_data();

        /*-----初步校准-----*/
        //等待二维码数据
	    while(Robot_Interface.Ar_Pose[grasp_ar_id[1]][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
	
        sx1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][0];
        sy1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][1];
        sz1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][2];
        ww1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][3];
        wx1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][4];
        wy1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][5];
        wz1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][6];
        
        std::cout << "转动后ar坐标：" <<std::endl;
        std::cout << "datax:" <<sx1<<std::endl;
        std::cout << "datay:" <<sy1<<std::endl;
        std::cout << "dataz:" <<sz1<<std::endl;

        Robot_Interface.Robot_MoveL(sx1-0.05,-sy1+0.02,0.0,robot);
        sleep(1.0);
        Robot_Interface.clean_ar_data();

        /*-----二次校准-----*/
        //等待二维码数据
	    while(Robot_Interface.Ar_Pose[grasp_ar_id[1]][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
	
        sx1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][0];
        sy1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][1];
        sz1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][2];
        ww1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][3];
        wx1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][4];
        wy1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][5];
        wz1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][6];
        
        std::cout << "初步平移校准后ar坐标" <<std::endl;
        std::cout << "datax:" <<sx1<<std::endl;
        std::cout << "datay:" <<sy1<<std::endl;
        std::cout << "dataz:" <<sz1<<std::endl;
        
        //  机械臂识别精度校准
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
        std::cout << "精度计算的坐标平移量" <<std::endl;
        std::cout << "datax:" <<sx1<<std::endl;
        std::cout << "datay:" <<sy1<<std::endl;
        std::cout << "dataz:" <<sz1<<std::endl;
        sleep(1.0);
        Robot_Interface.clean_ar_data();
        
        /*-----下去抓-----*/
        while(Robot_Interface.Ar_Pose[grasp_ar_id[1]][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
        sx1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][0];
        sy1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][1];
        sz1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][2];
        ww1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][3];
        wx1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][4];
        wy1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][5];
        wz1 = Robot_Interface.Ar_Pose[grasp_ar_id[1]][6];
        
        std::cout << "抓取前ar坐标" <<std::endl;
        std::cout << "datax:" <<sx1<<std::endl;
        std::cout << "datay:" <<sy1<<std::endl;
        std::cout << "dataz:" <<sz1<<std::endl;
        
        Robot_Interface.Robot_MoveL(sx1-grasp_x,-sy1+grasp_y,0.0,robot);
        Robot_Interface.Robot_MoveL(0.0,0.0,-sz1+grasp_z,robot);
        sleep(1.0);
        Robot_Interface.clean_ar_data();
        //关夹爪
	    bool close_state1 = Robot_Interface.Robot_Grasp_Control(0,robot);
        if(close_state1){
            //往上
            Robot_Interface.Robot_MoveL(0.0,0.0,graspid1_z,robot);
            Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_identify_pose,robot);
            //中间位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_fixed_middle_pose,robot);
            //初始位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
            //放置２号位姿
            Robot_Interface.Robot_MoveL(graspid1_x,graspid1_y,0.0,robot);
            //往下
            Robot_Interface.Robot_MoveL(0.0,0.0,-graspid1_z,robot);
        }
	    bool open_state1 = Robot_Interface.Robot_Grasp_Control(1,robot);
	    //往上
        Robot_Interface.Robot_MoveL(0.0,0.0,graspid1_z,robot);
        if(open_state1){
            //初始位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
        }
        std_msgs::Int32 arm_state;
        arm_state.data = 2;
        grasp_pub.publish(arm_state);

        /*--------------钣金件抓取---------------------*/
        // Robot_Interface.Robot_Set_Speed(move_sped,robot);

        
    /*-----------------------------id3, 钣金件-----------------------------*/
        while(AGV_Current_Goal != 2)
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
        //抓取中间位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.place_fixed_middle_pose,robot);
        //抓取识别位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.place_identify_pose,robot);
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

        if (move_left && !move_right)
        {
            std::cout<<"grasp target3 move left"<<std::endl;
            move_left = move_right = false;
        }else if(move_left && move_right)
        {
            std::cout<<"grasp target3 move right"<<std::endl;
            move_left = move_right = false;
        }

        sx = Robot_Interface.Ur_Pose[0][0];
        sy = Robot_Interface.Ur_Pose[0][1];
        sz = Robot_Interface.Ur_Pose[0][2];
        ww = Robot_Interface.Ur_Pose[0][3];
        wx = Robot_Interface.Ur_Pose[0][4];
        wy = Robot_Interface.Ur_Pose[0][5];
        wz = Robot_Interface.Ur_Pose[0][6];
        std::cout << "初步识别ar坐标：" <<std::endl;
        std::cout << "datax:" <<sx<<std::endl;
        std::cout << "datay:" <<sy<<std::endl;
        std::cout << "dataz:" <<sz<<std::endl;


        //构造四元数
        Eigen::Quaterniond quaternion_1(ww,wx,wy,wz);
        //四元数转欧拉角
        eulerAngle=quaternion_1.matrix().eulerAngles(2,1,0);
        //弧度转角度
        robot_angle = (eulerAngle(0) * 180) / PI;
        if(robot_angle > 90.0){
	            robot_angle = 180.0 - robot_angle;	
	    }
        if(robot_angle > 90){
            //  机械臂末端旋转
            Robot_Interface.Robot_MoveR(-PI/2 + eulerAngle(0), robot);
        }else{
            //  机械臂末端旋转
            Robot_Interface.Robot_MoveR(eulerAngle(0) - PI/2, robot);
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
	
        sx = Robot_Interface.Ur_Pose[0][0];
        sy = Robot_Interface.Ur_Pose[0][1];
        sz = Robot_Interface.Ur_Pose[0][2];
        ww = Robot_Interface.Ur_Pose[0][3];
        wx = Robot_Interface.Ur_Pose[0][4];
        wy = Robot_Interface.Ur_Pose[0][5];
        wz = Robot_Interface.Ur_Pose[0][6];
        
        std::cout << "转动后ar坐标：" <<std::endl;
        std::cout << "datax:" <<sx<<std::endl;
        std::cout << "datay:" <<sy<<std::endl;
        std::cout << "dataz:" <<sz<<std::endl;

        Robot_Interface.Robot_MoveL(sx-0.05,-sy+0.02,0.0,robot);
        sleep(1.0);
        Robot_Interface.clean_ur_data();

        /*-----二次校准-----*/
        //等待二维码数据
	    while(Robot_Interface.Ur_Pose[0][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
	
        sx = Robot_Interface.Ur_Pose[0][0];
        sy = Robot_Interface.Ur_Pose[0][1];
        sz = Robot_Interface.Ur_Pose[0][2];
        ww = Robot_Interface.Ur_Pose[0][3];
        wx = Robot_Interface.Ur_Pose[0][4];
        wy = Robot_Interface.Ur_Pose[0][5];
        wz = Robot_Interface.Ur_Pose[0][6];
        
        std::cout << "初步平移校准后ar坐标" <<std::endl;
        std::cout << "datax:" <<sx<<std::endl;
        std::cout << "datay:" <<sy<<std::endl;
        std::cout << "dataz:" <<sz<<std::endl;
        
        //  机械臂识别精度校准
        if(sy > 0.0616){
            sy = (sy - 0.0616);
            std::cout << sy <<std::endl;
        }
        else{
            sy = -(0.0616 - sy);
        }
        if(sx > 0.0035){
            sx = (sx - 0.0035);
        }
        else{
            sx = -(0.0035 - sx);
        }
        Robot_Interface.Robot_MoveL(sx,-sy,0.0,robot);
        std::cout << "精度计算的坐标平移量" <<std::endl;
        std::cout << "datax:" <<sx<<std::endl;
        std::cout << "datay:" <<sy<<std::endl;
        std::cout << "dataz:" <<sz<<std::endl;
        sleep(1.0);
        Robot_Interface.clean_ur_data();
        
        /*-----下去抓-----*/
        while(Robot_Interface.Ur_Pose[0][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
        sx = Robot_Interface.Ur_Pose[0][0];
        sy = Robot_Interface.Ur_Pose[0][1];
        sz = Robot_Interface.Ur_Pose[0][2];
        ww = Robot_Interface.Ur_Pose[0][3];
        wx = Robot_Interface.Ur_Pose[0][4];
        wy = Robot_Interface.Ur_Pose[0][5];
        wz = Robot_Interface.Ur_Pose[0][6];
        
        std::cout << "抓取前ar坐标" <<std::endl;
        std::cout << "datax:" <<sx<<std::endl;
        std::cout << "datay:" <<sy<<std::endl;
        std::cout << "dataz:" <<sz<<std::endl;
        
        Robot_Interface.Robot_MoveL(sx-bjj_grasp_x,-sy+bjj_grasp_y,0.0,robot);
        Robot_Interface.Robot_MoveL(0.0,0.0,-sz+bjj_grasp_z,robot);
        sleep(1.0);
        Robot_Interface.clean_ur_data();

        close_state = Robot_Interface.Robot_Grasp_Control(0,robot);
        if(close_state){
	        Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_identify_pose,robot);
            //中间位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_fixed_middle_pose,robot);
            //初始位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
            //放置２号位姿
            Robot_Interface.Robot_MoveL(bjj_place_x,bjj_place_y,0.0,robot);
            //往下
            Robot_Interface.Robot_MoveL(0.0,0.0,-bjj_place_z,robot);
        }
	    open_state = Robot_Interface.Robot_Grasp_Control(1,robot);
	    //往上
        Robot_Interface.Robot_MoveL(0.0,0.0,bjj_place_z,robot);
        if(open_state){
            //初始位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
        }
        //　发布抓取结束信号
        // std_msgs::Int32 arm_state;
        arm_state.data = 1;
        grasp_pub.publish(arm_state);

        /*-------------物料放置-------------*/
           //等待AGV到位信号
        while(AGV_Current_Goal != 3)
        {
            ros::spinOnce();
            loop_rate.sleep();
        }

    /*--------------------id1,螺母--------------------*/

        //抓取中间位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.place_fixed_middle_pose,robot);
        //抓取识别位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.place_identify_pose,robot);

        /*-----校准角度-----*/
	    while(Robot_Interface.Ar_Pose[LM_ID][0]==0.0){

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
        sx = Robot_Interface.Ar_Pose[LM_ID][0];
        sy = Robot_Interface.Ar_Pose[LM_ID][1];
        sz = Robot_Interface.Ar_Pose[LM_ID][2];
        ww = Robot_Interface.Ar_Pose[LM_ID][3];
        wx = Robot_Interface.Ar_Pose[LM_ID][4];
        wy = Robot_Interface.Ar_Pose[LM_ID][5];
        wz = Robot_Interface.Ar_Pose[LM_ID][6];
        
        std::cout << "初步识别ar坐标：" <<std::endl;
        std::cout << "datax:" <<sx<<std::endl;
        std::cout << "datay:" <<sy<<std::endl;
        std::cout << "dataz:" <<sz<<std::endl;


        //构造四元数
        Eigen::Quaterniond quaternion_2(ww,wx,wy,wz);
        //四元数转欧拉角
        eulerAngle=quaternion_2.matrix().eulerAngles(2,1,0);
        //弧度转角度
        robot_angle = (eulerAngle(0) * 180) / PI;
        if(robot_angle > 90.0){
	            robot_angle = 180.0 - robot_angle;	
	    }
        if(robot_angle > 90){
            //  机械臂末端旋转
            Robot_Interface.Robot_MoveR(-PI/2 + eulerAngle(0), robot);
            robot_angle = -PI/2 + eulerAngle(0);
        }else{
            //  机械臂末端旋转
            Robot_Interface.Robot_MoveR(eulerAngle(0) - PI/2, robot);
            robot_angle = eulerAngle(0) - PI/2;
        }

        std::cout<<"AR码角度： "<< robot_angle <<std::endl;
        sleep(1.0);
	    Robot_Interface.clean_ar_data();

        /*-----初步校准-----*/
        //等待二维码数据
	    while(Robot_Interface.Ar_Pose[LM_ID][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
	
        sx1 = Robot_Interface.Ar_Pose[LM_ID][0];
        sy1 = Robot_Interface.Ar_Pose[LM_ID][1];
        sz1 = Robot_Interface.Ar_Pose[LM_ID][2];
        ww1 = Robot_Interface.Ar_Pose[LM_ID][3];
        wx1 = Robot_Interface.Ar_Pose[LM_ID][4];
        wy1 = Robot_Interface.Ar_Pose[LM_ID][5];
        wz1 = Robot_Interface.Ar_Pose[LM_ID][6];
        
        std::cout << "转动后ar坐标：" <<std::endl;
        std::cout << "datax:" <<sx1<<std::endl;
        std::cout << "datay:" <<sy1<<std::endl;
        std::cout << "dataz:" <<sz1<<std::endl;

        Robot_Interface.Robot_MoveL(sx1-0.024,-sy1+0.02,0.0,robot);
        sleep(1.0);
        Robot_Interface.clean_ar_data();

        /*-----二次校准-----*/
        //等待二维码数据
	    while(Robot_Interface.Ar_Pose[LM_ID][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
	
        sx2 = Robot_Interface.Ar_Pose[LM_ID][0];
        sy2 = Robot_Interface.Ar_Pose[LM_ID][1];
        sz2 = Robot_Interface.Ar_Pose[LM_ID][2];
        ww2 = Robot_Interface.Ar_Pose[LM_ID][3];
        wx2 = Robot_Interface.Ar_Pose[LM_ID][4];
        wy2 = Robot_Interface.Ar_Pose[LM_ID][5];
        wz2 = Robot_Interface.Ar_Pose[LM_ID][6];
        
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
        Robot_Interface.clean_ar_data();
        
        /*-----往下-----*/
        while(Robot_Interface.Ar_Pose[LM_ID][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
        sx3 = Robot_Interface.Ar_Pose[LM_ID][0];
        sy3 = Robot_Interface.Ar_Pose[LM_ID][1];
        sz3 = Robot_Interface.Ar_Pose[LM_ID][2];
        ww3 = Robot_Interface.Ar_Pose[LM_ID][3];
        wx3 = Robot_Interface.Ar_Pose[LM_ID][4];
        wy3 = Robot_Interface.Ar_Pose[LM_ID][5];
        wz3 = Robot_Interface.Ar_Pose[LM_ID][6];
        
        std::cout << "抓取前ar坐标" <<std::endl;
        std::cout << "datax:" <<sx3<<std::endl;
        std::cout << "datay:" <<sy3<<std::endl;
        std::cout << "dataz:" <<sz3<<std::endl;
        
        Robot_Interface.Robot_MoveL(sx3-place_x,-sy3+place_y,-(sz3-place_z),robot);
        Robot_Interface.Robot_MoveL(0.0,0.0,sz3-place_z,robot);
        /*模拟动作结束, 开始复制动作放置*/
        sleep(1.0);
        Robot_Interface.clean_ar_data();

        //放置识别位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.place_identify_pose,robot);
        //放置中间位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.place_fixed_middle_pose,robot);
        //  物料盘上方
        Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
        //下去物料盘抓取螺母
        Robot_Interface.Robot_MoveL(-placeid2_x, placeid2_y, 0.0, robot);
        Robot_Interface.Robot_MoveL(0.0, 0.0, -placeid2_z, robot);
        //关夹爪
	    close_state = Robot_Interface.Robot_Grasp_Control(0,robot);
        if(close_state){
            //往上
            Robot_Interface.Robot_MoveL(0.0, 0.0, placeid2_z, robot);
            //物料盘上方
            Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
            //放置中间位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.place_fixed_middle_pose,robot);
            //放置识别位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.place_identify_pose,robot);
        }

        /*-----判断往左往右-----*/
        if (move_left && !move_right)
        {
            std::cout<<"move left"<<std::endl;
            Robot_Interface.Robot_MoveL(0.05,0.0,0.0,robot);
            move_left = move_right = false;
        }else if(move_left && move_right)
        {
            std::cout<<"move right"<<std::endl;
            Robot_Interface.Robot_MoveL(-0.05,0.0,0.0,robot);
            move_left = move_right = false;
        }

        Robot_Interface.Robot_MoveR(robot_angle, robot);
        Robot_Interface.Robot_MoveL(sx1-0.024,-sy1+0.02,0.0,robot);
        Robot_Interface.Robot_MoveL(sx2,-sy2,0.0,robot);
        Robot_Interface.Robot_MoveL(sx3-place_x,-sy3+place_y,0.0,robot);
        Robot_Interface.Robot_MoveL(0.0,0.0,-(sz3-place_z),robot);
	    open_state = Robot_Interface.Robot_Grasp_Control(1,robot);
        if(open_state){
            Robot_Interface.Robot_MoveL(0.0,0.0,sz3-place_z,robot);
            //放置识别位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.place_identify_pose,robot);
        }

/*-----------------------------id1 螺丝-----------------------------*/
        /*-----校准角度-----*/
	    while(Robot_Interface.Ar_Pose[LS_ID][0]==0.0){
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
        sx = Robot_Interface.Ar_Pose[LS_ID][0];
        sy = Robot_Interface.Ar_Pose[LS_ID][1];
        sz = Robot_Interface.Ar_Pose[LS_ID][2];
        ww = Robot_Interface.Ar_Pose[LS_ID][3];
        wx = Robot_Interface.Ar_Pose[LS_ID][4];
        wy = Robot_Interface.Ar_Pose[LS_ID][5];
        wz = Robot_Interface.Ar_Pose[LS_ID][6];
        
        std::cout << "初步识别ar坐标：" <<std::endl;
        std::cout << "datax:" <<sx<<std::endl;
        std::cout << "datay:" <<sy<<std::endl;
        std::cout << "dataz:" <<sz<<std::endl;


        //构造四元数
        Eigen::Quaterniond quaternion_3(ww,wx,wy,wz);
        //四元数转欧拉角
        eulerAngle1=quaternion_3.matrix().eulerAngles(2,1,0);
        //弧度转角度
        robot_angle = (eulerAngle1(0) * 180) / PI;
        if(robot_angle > 90.0){
	            robot_angle = 180.0 - robot_angle;	
	    }
        if(robot_angle > 90){
            //  机械臂末端旋转
            Robot_Interface.Robot_MoveR(-PI/2 + eulerAngle1(0), robot);
            robot_angle = -PI/2 + eulerAngle1(0);
        }else{
            //  机械臂末端旋转
            Robot_Interface.Robot_MoveR(eulerAngle1(0) - PI/2, robot);
            robot_angle = eulerAngle1(0) - PI/2;
        }

        std::cout<<"AR码角度： "<< robot_angle <<std::endl;
        sleep(1.0);
	    Robot_Interface.clean_ar_data();

        /*-----初步校准-----*/
        //等待二维码数据
	    while(Robot_Interface.Ar_Pose[LS_ID][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
	
        sx1 = Robot_Interface.Ar_Pose[LS_ID][0];
        sy1 = Robot_Interface.Ar_Pose[LS_ID][1];
        sz1 = Robot_Interface.Ar_Pose[LS_ID][2];
        ww1 = Robot_Interface.Ar_Pose[LS_ID][3];
        wx1 = Robot_Interface.Ar_Pose[LS_ID][4];
        wy1 = Robot_Interface.Ar_Pose[LS_ID][5];
        wz1 = Robot_Interface.Ar_Pose[LS_ID][6];
        
        std::cout << "转动后ar坐标：" <<std::endl;
        std::cout << "datax:" <<sx1<<std::endl;
        std::cout << "datay:" <<sy1<<std::endl;
        std::cout << "dataz:" <<sz1<<std::endl;

        Robot_Interface.Robot_MoveL(sx1-0.024,-sy1+0.02,0.0,robot);
        sleep(1.0);
        Robot_Interface.clean_ar_data();

        /*-----二次校准-----*/
        //等待二维码数据
	    while(Robot_Interface.Ar_Pose[LS_ID][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
	
        sx2 = Robot_Interface.Ar_Pose[LS_ID][0];
        sy2 = Robot_Interface.Ar_Pose[LS_ID][1];
        sz2 = Robot_Interface.Ar_Pose[LS_ID][2];
        ww2 = Robot_Interface.Ar_Pose[LS_ID][3];
        wx2 = Robot_Interface.Ar_Pose[LS_ID][4];
        wy2 = Robot_Interface.Ar_Pose[LS_ID][5];
        wz2 = Robot_Interface.Ar_Pose[LS_ID][6];
        
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
        Robot_Interface.clean_ar_data();
        
        /*-----往下-----*/
        while(Robot_Interface.Ar_Pose[LS_ID][0]==0.0){
                ros::spinOnce();
                loop_rate.sleep();
        }
        sx3 = Robot_Interface.Ar_Pose[LS_ID][0];
        sy3 = Robot_Interface.Ar_Pose[LS_ID][1];
        sz3 = Robot_Interface.Ar_Pose[LS_ID][2];
        ww3 = Robot_Interface.Ar_Pose[LS_ID][3];
        wx3 = Robot_Interface.Ar_Pose[LS_ID][4];
        wy3 = Robot_Interface.Ar_Pose[LS_ID][5];
        wz3 = Robot_Interface.Ar_Pose[LS_ID][6];
        
        std::cout << "抓取前ar坐标" <<std::endl;
        std::cout << "datax:" <<sx3<<std::endl;
        std::cout << "datay:" <<sy3<<std::endl;
        std::cout << "dataz:" <<sz3<<std::endl;
        
        Robot_Interface.Robot_MoveL(sx3-place_x,-sy3+place_y,-(sz3-place_z),robot);
        Robot_Interface.Robot_MoveL(0.0,0.0,sz3-place_z,robot);
        /*模拟动作结束, 开始复制动作放置*/
        sleep(1.0);
        Robot_Interface.clean_ar_data();

        //放置识别位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.place_identify_pose,robot);
        //放置中间位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.place_fixed_middle_pose,robot);
        //  物料盘上方
        Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
        //下去物料盘抓取螺母
        Robot_Interface.Robot_MoveL(-placeid1_x, placeid1_y, 0.0, robot);
        Robot_Interface.Robot_MoveL(0.0, 0.0, -placeid1_z, robot);
        //关夹爪
	    close_state1 = Robot_Interface.Robot_Grasp_Control(0,robot);
        if(close_state1){
            //往上
            Robot_Interface.Robot_MoveL(0.0, 0.0, placeid1_z, robot);
            //物料盘上方
            Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
            //放置中间位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.place_fixed_middle_pose,robot);
            //放置识别位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.place_identify_pose,robot);
        }
        /*-----判断往左往右-----*/
        if (move_left && !move_right)
        {
            std::cout<<"move left"<<std::endl;
            Robot_Interface.Robot_MoveL(0.05,0.0,0.0,robot);
            move_left = move_right = false;
        }else if(move_left && move_right)
        {
            std::cout<<"move right"<<std::endl;
            Robot_Interface.Robot_MoveL(-0.05,0.0,0.0,robot);
            move_left = move_right = false;
        }
        Robot_Interface.Robot_MoveR(robot_angle, robot);
        Robot_Interface.Robot_MoveL(sx1-0.024,-sy1+0.02,0.0,robot);
        Robot_Interface.Robot_MoveL(sx2,-sy2,0.0,robot);
        Robot_Interface.Robot_MoveL(sx3-place_x,-sy3+place_y,0.0,robot);
        Robot_Interface.Robot_MoveL(0.0,0.0,-(sz3-place_z),robot);
	    open_state1 = Robot_Interface.Robot_Grasp_Control(1,robot);
        if(open_state1){
            Robot_Interface.Robot_MoveL(0.0,0.0,sz3-place_z,robot);
            //放置识别位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.place_identify_pose,robot);
        }

/*-----------------------------id3, 钣金件-----------------------------*/
        // while(AGV_Current_Goal != 3)
        // {
        //     ros::spinOnce();
        //     loop_rate.sleep();
        // }
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
        sx = Robot_Interface.Ur_Pose[0][0];
        sy = Robot_Interface.Ur_Pose[0][1];
        sz = Robot_Interface.Ur_Pose[0][2];
        ww = Robot_Interface.Ur_Pose[0][3];
        wx = Robot_Interface.Ur_Pose[0][4];
        wy = Robot_Interface.Ur_Pose[0][5];
        wz = Robot_Interface.Ur_Pose[0][6];
        
        std::cout << "初步识别ar坐标：" <<std::endl;
        std::cout << "datax:" <<sx<<std::endl;
        std::cout << "datay:" <<sy<<std::endl;
        std::cout << "dataz:" <<sz<<std::endl;


        //构造四元数
        Eigen::Quaterniond quaternion2(ww,wx,wy,wz);
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
            robot_angle = -PI/2 + eulerAngle2(0);
        }else{
            //  机械臂末端旋转
            Robot_Interface.Robot_MoveR(eulerAngle2(0) - PI/2, robot);
            robot_angle = eulerAngle2(0)- PI/2;
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
        
        std::cout << "抓取前ar坐标" <<std::endl;
        std::cout << "datax:" <<sx3<<std::endl;
        std::cout << "datay:" <<sy3<<std::endl;
        std::cout << "dataz:" <<sz3<<std::endl;
        
        Robot_Interface.Robot_MoveL(sx3-zpt_bjj_place_x,-sy3+zpt_bjj_place_y,-(sz3-zpt_bjj_place_z),robot);
        Robot_Interface.Robot_MoveL(0.0,0.0,sz3-zpt_bjj_place_z,robot);
        /*模拟动作结束, 开始复制动作放置*/
        sleep(1.0);
        Robot_Interface.clean_ur_data();

        //放置识别位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.place_identify_pose,robot);
        //放置中间位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.place_fixed_middle_pose,robot);
        //  物料盘上方
        Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
        //下去物料盘抓取螺母
        Robot_Interface.Robot_MoveL(-zpt_bjj_grasp_x, zpt_bjj_grasp_y, 0.0, robot);
        Robot_Interface.Robot_MoveL(0.0, 0.0, -zpt_bjj_grasp_z, robot);
        //关夹爪
	    bool close_state2 = Robot_Interface.Robot_Grasp_Control(0,robot);
        if(close_state2){
            //往上
            Robot_Interface.Robot_MoveL(0.0, 0.0, zpt_bjj_grasp_z, robot);
            //物料盘上方
            Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
            //放置中间位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.place_fixed_middle_pose,robot);
            //放置识别位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.place_identify_pose,robot);
        }
         /*-----判断往左往右-----*/
        if (move_left && !move_right)
        {
            std::cout<<"move left"<<std::endl;
            Robot_Interface.Robot_MoveL(0.05,0.0,0.0,robot);
            move_left = move_right = false;
        }else if(move_left && move_right)
        {
            std::cout<<"move right"<<std::endl;
            Robot_Interface.Robot_MoveL(-0.05,0.0,0.0,robot);
            move_left = move_right = false;
        }
        Robot_Interface.Robot_MoveR(robot_angle, robot);
        Robot_Interface.Robot_MoveL(sx1-0.024,-sy1+0.02,0.0,robot);
        Robot_Interface.Robot_MoveL(sx2,-sy2,0.0,robot);
        Robot_Interface.Robot_MoveL(sx3-zpt_bjj_place_x,-sy3+zpt_bjj_place_y,0.0,robot);
        Robot_Interface.Robot_MoveL(0.0,0.0,-(sz3-zpt_bjj_place_z),robot);
	    bool open_state2 = Robot_Interface.Robot_Grasp_Control(1,robot);
        if(open_state2){
            Robot_Interface.Robot_MoveL(0.0,0.0,sz3-zpt_bjj_place_z,robot);
            //放置识别位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.place_identify_pose,robot);
	        Robot_Interface.Robot_MoveJ(Robot_Interface.place_fixed_middle_pose,robot);
        }
        //　发布放置s结束信号
        tag.data = 1;
        place_pub.publish(tag);
        tag.data = 0;

        /*---------------成品抓取------------------*/
        //等待信号
        while(ZPT_Current_State != 1)
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
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
        Eigen::Quaterniond quaternion_4(ww1,wx1,wy1,wz1);
        //四元数转欧拉角
        eulerAngle2=quaternion_4.matrix().eulerAngles(2,1,0);
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
        
        std::cout << "抓取前ar坐标" <<std::endl;
        std::cout << "datax:" <<sx3<<std::endl;
        std::cout << "datay:" <<sy3<<std::endl;
        std::cout << "dataz:" <<sz3<<std::endl;

        // Robot_Interface.Robot_MoveL(sx3-bjj_place_x,-sy3+bjj_place_y,-(sz3-bjj_place_z),robot);
        Robot_Interface.Robot_MoveL(sx3-cp_bjj_place_x,-sy3+cp_bjj_place_y,0.0,robot);
	    Robot_Interface.Robot_MoveL(0.0,0.0,-(sz3-cp_bjj_place_z),robot);
        // Robot_Interface.Robot_MoveL(0.0,0.0,sz3-bjj_place_z,robot);
        close_state2 = Robot_Interface.Robot_Grasp_Control(0,robot);
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
        /*模拟动作结束, 开始复制动作放置*/
        sleep(1.0);
        Robot_Interface.clean_ar_data();
	    Robot_Interface.Robot_MoveL(-cp_bjj_grasp_x, cp_bjj_grasp_y, 0.0, robot);
        Robot_Interface.Robot_MoveL(0.0, 0.0, -cp_bjj_grasp_z, robot);
        //开夹爪
	    open_state2 = Robot_Interface.Robot_Grasp_Control(1,robot);
        if(open_state2){
            //往上
            Robot_Interface.Robot_MoveL(0.0, 0.0, cp_bjj_grasp_z, robot);
            //物料盘上方
            Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
        }
        //  发布抓取成品完成信号
        tag.data = 1;
        assemble_pub.publish(tag);
        tag.data = 0;
        //等待AGV到位信号
        while(AGV_Current_Goal != 4)
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
        //抓取中间位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_fixed_middle_pose,robot);
        //抓取识别位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_pubsh_pose,robot);

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
        
        Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_id1_pubsh,robot);
        Robot_Interface.Robot_MoveL(-product_grasp_x,product_grasp_y,0.0,robot);

        Robot_Interface.Robot_MoveL(0.0,0.0,-product_grasp_z,robot);
        close_state1 = Robot_Interface.Robot_Grasp_Control(0,robot);
        if(close_state1){
            Robot_Interface.Robot_MoveL(0.0,0.0,product_grasp_z,robot);
            //抓取中间位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_fixed_middle_pose,robot);
            //抓取识别位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_pubsh_pose,robot);
        }
        /*Robot_Interface.Robot_MoveL(sx-0.05,-sy+0.02,0.0,robot);
        Robot_Interface.Robot_MoveL(sx1,-sy1,0.0,robot);
        Robot_Interface.Robot_MoveL(0.0,-0.03,0.0,robot);
        Robot_Interface.Robot_MoveL(0.0,0.0,-sz1+0.25,robot);
        Robot_Interface.Robot_MoveR(-PI/2, robot);
        Robot_Interface.Robot_MoveL(0.06 + Ste_warehouse_x, 0.11 + Ste_warehouse_y, 0.0, robot);
        Robot_Interface.Robot_MoveL(0.0, 0.0, -0.02 + Ste_warehouse_z,robot);*/
	Robot_Interface.Robot_MoveL(sx-0.05,-sy+0.02,0.0,robot);
        Robot_Interface.Robot_MoveL(sx1,-sy1,0.0,robot);
        Robot_Interface.Robot_MoveL(Ste_warehouse_x, Ste_warehouse_y, 0.0, robot);
        Robot_Interface.Robot_MoveL(0.0, 0.0, Ste_warehouse_z,robot);
        open_state = Robot_Interface.Robot_Grasp_Control(1,robot);
        if(open_state){
            Robot_Interface.Robot_MoveL(0.0,0.0,0.04,robot);
            //放置识别位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.LTCK_fixed_middle_pose,robot);
        }
        Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
    }catch (xmate::ControlException &e) {
        std::cout << e.what() << std::endl;
    }
    /*---------抓取2号结束----------*/
    /*---------华丽的分割线---------*/
    
    ros::spin();
    return 0;
    

}
