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
    //监听ar码
    ros::Subscriber ur_marker_sub = n.subscribe("/aruco_single/pose", 1, &HG_AI_Robot::Get_UR_Pose, &Robot_Interface);

    //判断上电状态，未上电则上电Start_Moment_Thread
    if(!power_state)
    {
        Robot_Interface.Robot_Set_Power(1,robot);
    }

    //初始化参数
 	float sx ,sy ,sz ,ww ,wx ,wy, wz;
    //  机械臂平移运动
	float sx1 ,sy1 ,sz1 ,ww1 ,wx1 ,wy1, wz1;
    //  机械臂平移运动校准
    float sx2 ,sy2 ,sz2 ,ww2 ,wx2 ,wy2, wz2;
     //  机械臂平移运动放置
    float sx3 ,sy3 ,sz3 ,ww3 ,wx3 ,wy3, wz3;

    //放置顺序对应的AR码ID
    int LS_ID = 0;
    int LM_ID = 1;
    int BJJ_ID = 2;

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

    //  从物料盘上抓取成品
    double product_grasp_x, product_grasp_y, product_grasp_z; 

    //钣金件放置抓取参数
    double zpt_bjj_place_x,zpt_bjj_place_y,zpt_bjj_place_z;
    double zpt_bjj_grasp_x,zpt_bjj_grasp_y,zpt_bjj_grasp_z;

    double robot_angle = 0.0;   // 校准角度

    //放置完成信号
    std_msgs::Int32 tag;

     //未接收到数据次数
    int no_data_time = 0;
    bool move_left = false;
    bool move_right = false;

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


    // 物料id1从物料盘上面抓取参数
    node.param("placeid1_x", placeid1_x, 0.015);  //  从物料盘抓取id1物料时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("placeid1_y", placeid1_y, 0.004);  //　从物料盘抓取id1物料时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("placeid1_z", placeid1_z, 0.040134663);  //　从物料盘抓取id1物料时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小

    // 物料id2从物料盘上面抓取参数
    node.param("placeid2_x", placeid2_x, 0.0);  //  从物料盘抓取id2物料时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("placeid2_y", placeid2_y, 0.1115);  //　从物料盘抓取id2物料时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("placeid2_z", placeid2_z, 0.041234663);  //　从物料盘抓取id2物料时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小

    //  机械臂放置物料完成标志位
    ros::Publisher place_pub = n.advertise<std_msgs::Int32>("PlaceDone", 1000);
    //  成品抓取
    ros::Publisher assemble_pub = n.advertise<std_msgs::Int32>("AssembleDone", 1000);

    //速度限制
    move_sped = min(move_sped, 0.3);

    ros::Rate loop_rate(10);

	ros::Subscriber agv_flag = n.subscribe("agv_state", 1, AGV_CallBack);
    Robot_Interface.Stop_Moment_Thread = true;
    //等待AGV到位信号
	while(AGV_Current_Goal != 2)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

/*--------------------id1,螺母--------------------*/

    try{
	    Robot_Interface.Robot_Set_Speed(move_sped,robot);
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
        //Robot_Interface.clean_ar_data();
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
        //Robot_Interface.clean_ar_data();
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
        //Robot_Interface.clean_ar_data();
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
	    bool close_state = Robot_Interface.Robot_Grasp_Control(0,robot);
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
	    bool open_state = Robot_Interface.Robot_Grasp_Control(1,robot);
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
        Eigen::Quaterniond quaternion1(ww,wx,wy,wz);
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
        //Robot_Interface.clean_ar_data();
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
        //Robot_Interface.clean_ar_data();
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
        //Robot_Interface.clean_ar_data();
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
	    bool close_state1 = Robot_Interface.Robot_Grasp_Control(0,robot);
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
	    bool open_state1 = Robot_Interface.Robot_Grasp_Control(1,robot);
        if(open_state1){
            Robot_Interface.Robot_MoveL(0.0,0.0,sz3-place_z,robot);
            //放置识别位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.place_identify_pose,robot);
            //放置中间位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.place_fixed_middle_pose,robot);
            //  物料盘上方
            Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
        }

/*-----------------------------id3, 钣金件-----------------------------*/
        while(AGV_Current_Goal != 3)
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
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
	    Robot_Interface.clean_ar_data();

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
        //Robot_Interface.clean_ar_data();
        std::cout << "转动后ar坐标：" <<std::endl;
        std::cout << "datax:" <<sx1<<std::endl;
        std::cout << "datay:" <<sy1<<std::endl;
        std::cout << "dataz:" <<sz1<<std::endl;

        Robot_Interface.Robot_MoveL(sx1-0.024,-sy1+0.02,0.0,robot);
        sleep(1.0);
        Robot_Interface.clean_ar_data();

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
        //Robot_Interface.clean_ar_data();
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
        //Robot_Interface.clean_ar_data();
        std::cout << "抓取前ar坐标" <<std::endl;
        std::cout << "datax:" <<sx3<<std::endl;
        std::cout << "datay:" <<sy3<<std::endl;
        std::cout << "dataz:" <<sz3<<std::endl;
        
        Robot_Interface.Robot_MoveL(sx3-zpt_bjj_place_x,-sy3+zpt_bjj_place_y,-(sz3-zpt_bjj_place_z),robot);
        Robot_Interface.Robot_MoveL(0.0,0.0,sz3-zpt_bjj_place_z,robot);
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

    }catch (xmate::ControlException &e) {
        std::cout << e.what() << std::endl;
    }
    /*---------抓取2号结束----------*/
    /*---------华丽的分割线---------*/
    
    //　发布放置s结束信号
    tag.data = 1;
    place_pub.publish(tag);
    tag.data = 0;
    ros::spin();
    return 0;
}
