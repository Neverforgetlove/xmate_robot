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
    double move_dev = 0.05;
    bool ls_grasp_state = false;
    bool lm_grasp_state = false;
    bool start_moment = false;
    //机械臂移动速度
    node.param("move_sped", move_sped, 0.2);
    //启动力矩监听
    node.param("monitor_state", start_moment, false);
    // 偏移距离
    node.param("move_deviation", move_dev, 0.05);

    if(move_dev>0.1){
        move_dev = 0.1;
    }
    if(move_dev<0.01)
    {
        move_dev = 0.01;
    }

    //未接收到数据次数
    int no_data_time = 0;
    bool move_left = false;
    bool move_right = false;

    //初始化
    HG_AI_Robot Robot_Interface;
    Robot_Interface.Jog0_Robot_Moment = 3;
    Robot_Interface.Jog4_Robot_Moment = 25;
    Robot_Interface.Start_Moment_Thread = start_moment;

    Robot_Interface.Move_Deviation = move_dev;
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

    // 机械臂抓取物料完成标志位
    ros::Publisher grasp_pub = n.advertise<std_msgs::Int32>("GraspDone", 1000);


    //初始化参数
 	float sx ,sy ,sz ,ww ,wx ,wy, wz;
	float sx1 ,sy1 ,sz1 ,ww1 ,wx1 ,wy1, wz1;

    //抓取物料类型设置
    int grasp_type = 1;
    //抓取类型对应的二维码号, 螺丝螺母钣金件
    int grasp_ar_id[3] = {0,1,2}; 

    //  抓取物料偏移量
	double grasp_x, grasp_y, grasp_z;

    //  将id1物料放置到物料盘上偏移量
	double graspid1_x, graspid1_y, graspid1_z;
    //  将id2物料放置到物料盘上偏移量
	double graspid2_x, graspid2_y, graspid2_z;

    //钣金件抓取放置偏移量
    double bjj_grasp_x, bjj_grasp_y, bjj_grasp_z;
    double bjj_graspid1_x, bjj_graspid1_y, bjj_graspid1_z;
    

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
    
    //机械臂抓取类型
    node.param("target_type", grasp_type, 1);

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

    //抓取参数限制
    // grasp_x = min(grasp_x, 0.02);
    // grasp_y = min(grasp_y, 0.07);
    // grasp_z = min(grasp_z, 0.25);

    //物料id1放置限制
    // graspid1_x = min(graspid1_x, 0.02);
    // graspid1_y = min(graspid1_y, 0.07);
    // graspid1_z = min(graspid1_z, 0.07);

    //物料id2放置限制
    // graspid2_x = min(grasp_x, 0.02);
    // grasp_x = min(graspid2_y, 0.13);
    // graspid2_z = min(graspid2_z, 0.04);

    ros::Rate loop_rate(10);

	ros::Subscriber agv_flag = n.subscribe("agv_state", 1, AGV_CallBack);
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
        //清除之前识别到的无用数据
        Robot_Interface.clean_ar_data();
        Robot_Interface.clean_ur_data();
        sleep(0.5);
        /*-----校准角度-----*/
	    while(Robot_Interface.Ar_Pose[grasp_ar_id[2]][0]==0.0){
            no_data_time +=1;
            if(no_data_time>5 && !move_left){
                Robot_Interface.Robot_MoveL(Robot_Interface.Move_Deviation,0.0,0.0,robot);
                move_left = true;
                no_data_time = 0;
            }else if(no_data_time > 5 && !move_right){
                Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_identify_pose,robot);
                Robot_Interface.Robot_MoveL(-Robot_Interface.Move_Deviation,0.0,0.0,robot);
                move_right = true;
                no_data_time = 0;
            }else if(move_left && move_right){
                break;
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
        if(Robot_Interface.Ar_Pose[grasp_ar_id[2]][0] !=0.0){
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
            //Robot_Interface.clean_ar_data();
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
            //Robot_Interface.clean_ar_data();
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
            //Robot_Interface.clean_ar_data();
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
                ls_grasp_state = true;
            }
        }
/*-----------------------------id1 螺母-----------------------------*/
        //等待AGV到位信号
        // while(AGV_Current_Goal != 2)
        // {
        //     ros::spinOnce();
        //     loop_rate.sleep();
        // }
        //速度
        Robot_Interface.Robot_Set_Speed(move_sped,robot);
        if(ls_grasp_state){
            //抓取中间位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_fixed_middle_pose,robot);
            //抓取识别位姿
            Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_identify_pose,robot);
        }
        //清除之前识别到的无用数据
        Robot_Interface.clean_ar_data();
        Robot_Interface.clean_ur_data();
        sleep(0.5);
        /*-----校准角度-----*/
	    while(Robot_Interface.Ar_Pose[grasp_ar_id[1]][0]==0.0){
            no_data_time +=1;
            if(no_data_time>5 && !move_left){
                Robot_Interface.Robot_MoveL(Robot_Interface.Move_Deviation,0.0,0.0,robot);
                move_left = true;
                no_data_time = 0;
            }else if(no_data_time > 5 && !move_right){
                Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_identify_pose,robot);
                Robot_Interface.Robot_MoveL(-Robot_Interface.Move_Deviation,0.0,0.0,robot);
                move_right = true;
                no_data_time = 0;
            }else if(move_right and move_left){
                break;
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

        if(Robot_Interface.Ar_Pose[grasp_ar_id[1]][0]!=0.0){
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
            //Robot_Interface.clean_ar_data();
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
            //Robot_Interface.clean_ar_data();
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
            //Robot_Interface.clean_ar_data();
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
                Robot_Interface.Robot_MoveL(0.0,0.0,graspid2_z,robot);
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
                lm_grasp_state = true;
            }
        }
        if(ls_grasp_state==false || lm_grasp_state == false){
            Robot_Interface.Robot_MoveJ(Robot_Interface.grasp_fixed_middle_pose,robot);
            Robot_Interface.Robot_MoveJ(Robot_Interface.pubsh_pose,robot);
        }
    }catch (xmate::ControlException &e) {
        std::cout << e.what() << std::endl;
    }
    /*---------抓取2号结束----------*/
    /*---------华丽的分割线---------*/
    
    //　发布抓取结束信号
    std_msgs::Int32 arm_state;
    arm_state.data = 1;
    grasp_pub.publish(arm_state);
    ros::spin();
    return 0;
}
