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
    ros::Subscriber marker_sub = n.subscribe("/aruco_single/pose", 1, &HG_AI_Robot::Get_UR_Pose, &Robot_Interface);
    ros::Subscriber marker_sub2 = n.subscribe("/aruco_single/pose2", 1, &HG_AI_Robot::Get_UR_Pose2, &Robot_Interface);
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
    int BJJ_ID = 4;

    //钣金件放置抓取参数
    double bjj_place_x,bjj_place_y,bjj_place_z;
    double bjj_grasp_x,bjj_grasp_y,bjj_grasp_z;

    double robot_angle = 0.0;   // 校准角度

    //放置完成信号
    std_msgs::Int32 tag;

    //定义PI和固定位姿
    const double PI = 3.14159;
    std::array<double, 7> q_init;

    // // 机械臂抓取识别位姿
    // std::array<double, 7> grasp_identify_pose = {{(-95.365406 * PI / 180), (-14.531547* PI / 180), (5.96469726 * PI / 180), (-69.987455 * PI / 180), (-2.7401447 * PI / 180), (-94.285182 * PI / 180), (-4.0333042 * PI / 180)}};

    // // 机械臂放置到物料盘与物料台中转位置
    // std::array<double, 7> grasp_fixed_middle_pose = {{(-30.278141 * PI / 180), (-14.834426* PI / 180), (6.0488525 * PI / 180), (-74.248970 * PI / 180), (-2.8477249 * PI / 180), (-90.563409 * PI / 180), (-2.7336559 * PI / 180)}};

    // // 物料放置位置
    // std::array<double,7> grasp_id1_pubsh = {{(-18.750915 * PI / 180), (-5.1412582 * PI / 180), (3.69308166 * PI / 180), (-101.94888 * PI / 180), (-1.8495998 * PI / 180), (-72.657669 * PI / 180), (-15.904083 * PI / 180)}};
    // std::array<double,7> pubsh_pose = {{(-18.750915 * PI / 180), (-5.1412582 * PI / 180), (3.69308166 * PI / 180), (-101.94888 * PI / 180), (-1.8495998 * PI / 180), (-72.657669 * PI / 180), (-15.904083 * PI / 180)}};
    
    // //机械臂识别位姿
    // std::array<double, 7> place_identify_pose = {{(-95.365406 * PI / 180), (-14.531547* PI / 180), (5.96469726 * PI / 180), (-69.987455 * PI / 180), (-2.7401447 * PI / 180), (-94.285182 * PI / 180), (-4.0333042 * PI / 180)}};
    // // 机械臂放置到物料盘与装配台中转位置
    // std::array<double, 7> place_fixed_middle_pose = {{(-30.278141 * PI / 180), (-14.834426* PI / 180), (6.0488525 * PI / 180), (-74.248970 * PI / 180), (-2.8477249 * PI / 180), (-90.563409 * PI / 180), (-2.7336559 * PI / 180)}};

    // 从物料盘上面抓取钣金件参数
    node.param("bjj_grasp_x", bjj_grasp_x, 0.015);  //  从物料盘抓取id1物料时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左
    node.param("bjj_grasp_y", bjj_grasp_y, 0.004);  //　从物料盘抓取id1物料时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前
    node.param("bjj_grasp_z", bjj_grasp_z, 0.040134663);  //　从物料盘抓取id1物料时机械臂的z坐标偏移量(笛卡尔坐标系)，往下减小

    // 设置放置钣金件参数
    node.param("bjj_place_x", bjj_place_x, 0.007);  // 放置物料到装配台时机械臂的x坐标偏移量(笛卡尔坐标系)，增大往左(默认：0.007)
    node.param("bjj_place_y", bjj_place_y, 0.053);  //　放置物料到装配台时机械臂的y坐标偏移量(笛卡尔坐标系)，增大向前(默认：0.053)
    node.param("bjj_place_z", bjj_place_z, 0.253);    //　放置物料到装配台时机械臂的z坐标偏移量(笛卡尔坐标系)，往下增大(默认：0.253)

    //  机械臂放置物料完成标志位
    ros::Publisher place_pub = n.advertise<std_msgs::Int32>("PlaceDone", 1000);
    //  成品抓取
    ros::Publisher assemble_pub = n.advertise<std_msgs::Int32>("AssembleDone", 1000);

    //速度限制
    move_sped = min(move_sped, 0.3);

    ros::Rate loop_rate(10);

	ros::Subscriber agv_flag = n.subscribe("agv_state", 1, AGV_CallBack);
    Robot_Interface.Stop_Moment_Thread = true;

/*--------------------id1,螺母--------------------*/

    try{
	    Robot_Interface.Robot_Set_Speed(move_sped,robot);
        //抓取中间位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.place_fixed_middle_pose,robot);
        //抓取识别位姿
        Robot_Interface.Robot_MoveJ(Robot_Interface.place_identify_pose,robot);

        
/*-----------------------------id3, 钣金件-----------------------------*/
        while(AGV_Current_Goal != 3)
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
        //清除之前识别到的无用数据
        Robot_Interface.clean_ar_data();
        Robot_Interface.clean_ur_data();
        sleep(0.5);
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
            std::cout<<"move left"<<std::endl;
            move_left = move_right = false;
        }else if(move_left && move_right)
        {
            std::cout<<"move right"<<std::endl;
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
        //Robot_Interface.clean_ur_data();
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
        //Robot_Interface.clean_ur_data();
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
        //Robot_Interface.clean_ur_data();
        std::cout << "抓取前ar坐标" <<std::endl;
        std::cout << "datax:" <<sx<<std::endl;
        std::cout << "datay:" <<sy<<std::endl;
        std::cout << "dataz:" <<sz<<std::endl;
        
        Robot_Interface.Robot_MoveL(sx-bjj_grasp_x,-sy+bjj_grasp_y,0.0,robot);
        Robot_Interface.Robot_MoveL(0.0,0.0,-sz+bjj_grasp_z,robot);
        sleep(1.0);
        Robot_Interface.clean_ur_data();

        bool close_state = Robot_Interface.Robot_Grasp_Control(0,robot);
        if(close_state){
            //往上
            Robot_Interface.Robot_MoveL(0.0,0.0,bjj_place_z,robot);
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
	    bool open_state = Robot_Interface.Robot_Grasp_Control(1,robot);
	    //往上
        Robot_Interface.Robot_MoveL(0.0,0.0,bjj_place_z,robot);
        if(open_state){
            //初始位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
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
