#include "hg_ai_robot.h"

HG_AI_Robot::HG_AI_Robot()
{   
    std::cout<<"HG_AI_Robot"<<std::endl;
};

HG_AI_Robot::~HG_AI_Robot(){}

//监听力矩
void HG_AI_Robot::Check_Robot_Moment(xmate::Robot& robot)
{   
    while(1){
        if(!Stop_Moment_Thread)
        {
            //获取关节力矩
            current_robot_moment = robot.receiveRobotState().tau_m;
            std::cout<<"Robot_moment: "<<current_robot_moment<<std::endl;
            if(current_robot_moment[3]<Jog4_Robot_Moment || abs(current_robot_moment[0])>Jog0_Robot_Moment){
                robot.stopMove();
                sleep(1.5);
                Robot_Set_Power(0, robot); //力矩异常下电
                std::cout<<"Robot error, Power OFF!!!"<<std::endl;
                return;
            }
        }else{
            sleep(1.0);
        }
        // limit_pose.pos = robot.receiveRobotState().toolTobase_pos_m; //更新位姿
        // std::cout<<"Robot Pose: "<<limit_pose.pos[3]<<","<<limit_pose.pos[7]<<","<<limit_pose.pos[11]<<std::endl;
 
        // limit_pose.pos = robot.receiveRobotState().toolTobase_pos_m; //更新位姿
        // if(limit_pose.pos[11]<0.2){
        //     robot.stopMove();
        //     Robot_Set_Power(0, robot); //位姿超限下电
        //     std::cout<<"Robot Pose error, Power OFF!!!"<<std::endl;
        //     return;
        // }
        
	    sleep(1.0);
    }
};

//初始化函数
void HG_AI_Robot::Init(xmate::Robot& robot)
{   
    Robot_Get_Power(robot);
    if(!RobotPower){
        Robot_Set_Power(1, robot);
    }
    Robot_Get_Joint(robot);
    Robot_Get_State(robot);
    Robot_Grasp_Control(1,robot);
    sleep(1.0);
    if(Start_Moment_Thread){
        Listen_Moment = std::thread(&HG_AI_Robot::Check_Robot_Moment, this, std::ref(robot));
        Listen_Moment.detach();
    }
};

//更新关节矩阵
void HG_AI_Robot::Robot_Get_Joint(xmate::Robot& robot)
{
   robot_joint_state = robot.receiveRobotState().q;
   std::cout<<"Update Robot Joint!!"<<std::endl;
};

//更新机械臂状态
void HG_AI_Robot::Robot_Get_State(xmate::Robot& robot)
{
    robot_state = robot.receiveRobotState(); //更新状态
    current_pose.pos = robot_state.toolTobase_pos_m; //更新位姿
    end_pose.pos  = robot_state.toolTobase_pos_m; //更新位姿
    std::cout<<"Update Robot Pose!!"<<std::endl;
};

//MoveJ指令
bool HG_AI_Robot::Robot_MoveJ(std::array<double, 7>target_jont, xmate::Robot& robot)
{   
    Robot_Get_Joint(robot);
    MOVEJ(Move_Speed, robot_joint_state, target_jont, robot);
    return true;
};

//MoveL指令
bool HG_AI_Robot::Robot_MoveL(float x, float y, float z, xmate::Robot& robot)
{   
    Robot_Get_State(robot);
    end_pose.pos[3] += x;
    end_pose.pos[7] += y;
    end_pose.pos[11] += z;
    MOVEL(Move_Speed, current_pose, end_pose, robot);
    return true;
};

//x方向移动
bool HG_AI_Robot::Robot_MoveX(float x, xmate::Robot& robot)
{
    Robot_Get_State(robot);
    end_pose.pos[3] += x;
    MOVEL(Move_Speed, current_pose, end_pose, robot);
    return true;
};

//y方向移动
bool HG_AI_Robot::Robot_MoveY(float y, xmate::Robot& robot)
{ 
    Robot_Get_State(robot);
    end_pose.pos[7] += y;
    MOVEL(Move_Speed, current_pose, end_pose, robot);
    return true;
};

//z方向移动
bool HG_AI_Robot::Robot_MoveZ(float z, xmate::Robot& robot)
{ 
    Robot_Get_State(robot);
    end_pose.pos[11] += z;
    MOVEL(Move_Speed, current_pose, end_pose, robot);
    return true;
};

//MoveR指令
bool HG_AI_Robot::Robot_MoveR(float roation_z, xmate::Robot& robot)
{
    //  实时获取机器人当前关节角
    Robot_Get_State(robot);
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
    MOVEL(Move_Speed, current_pose, end_pose, robot);
    Robot_Get_State(robot);
    //sleep(1.0);
    return true;
};

//设置DO状态
bool HG_AI_Robot::Robot_Set_DO(int DO_ID, int DO_State, xmate::Robot& robot)
{
    switch(DO_ID){
        case 0:
            DO_signal = RCI::robot::DOSIGNAL::DO0_0;
            break;
        case 1:
            DO_signal = RCI::robot::DOSIGNAL::DO0_1;
            break;
        case 2:
            DO_signal = RCI::robot::DOSIGNAL::DO0_2;
            break;
        case 3:
            DO_signal = RCI::robot::DOSIGNAL::DO0_3;
            break;
        case 4:
            DO_signal = RCI::robot::DOSIGNAL::DO1_0;
            break;
        case 5:
            DO_signal = RCI::robot::DOSIGNAL::DO1_1;
            break;
        default:
            std::cout<<" SET DO ID ERROR!!!"<<std::endl;
            return false;
    }
    robot.setDO(DO_signal,DO_State);
    return true;
};

//获取DI状态
int HG_AI_Robot::Robot_Get_DI(int DI_ID, xmate::Robot& robot)
{
    switch(DI_ID){
        case 0:
            DI_signal = RCI::robot::DISIGNAL::DI0_0;
            break;
        case 1:
            DI_signal = RCI::robot::DISIGNAL::DI0_1;
            break;
        case 2:
            DI_signal = RCI::robot::DISIGNAL::DI0_2;
            break;
        case 3:
            DI_signal = RCI::robot::DISIGNAL::DI0_3;
            break;
        case 4:
            DI_signal = RCI::robot::DISIGNAL::DI1_0;
            break;
        case 5:
            DI_signal = RCI::robot::DISIGNAL::DI1_1;
            break;
        default:
            std::cout<<"DI ID ERROR!!!"<<std::endl;
            return false;
    }
    return robot.getDI(DI_signal);
};

// 机器人控制末端夹抓 1-开 0-关
bool HG_AI_Robot::Robot_Grasp_Control(int grasp_state, xmate::Robot& robot)
{
    if(grasp_state){
    	Robot_Set_DO(1,1,robot);
        Robot_Set_DO(3,1,robot);
        sleep(1.0);
        if(Robot_Get_DI(5,robot)){
            std::cout<<"Robot Grasp Open"<<std::endl;
            Robot_Set_DO(1,0,robot);
            Robot_Set_DO(3,0,robot);
            return true;
        }else{
            return false;
        }
    }else{
        Robot_Set_DO(0,1,robot);
        Robot_Set_DO(3,1,robot);
        sleep(1.0);
        if(Robot_Get_DI(5,robot)){
	        std::cout<<"Robot Grasp Close"<<std::endl;
            Robot_Set_DO(0,0,robot);
            Robot_Set_DO(3,0,robot);
            return true;
        }else{
            return false;
        }
    }
    return false;
};

//机器人电源控制
bool HG_AI_Robot::Robot_Set_Power(int power, xmate::Robot& robot)
{   
    if(power == 1){
        std::cout<<"Robot Power ON!!"<<std::endl;
    }
    if(power == 0){
        std::cout<<"Robot Power OFF!!"<<std::endl;
    }
    robot.setMotorPower(power);
    return true;
};

//获取机器人上电状态
int HG_AI_Robot::Robot_Get_Power(xmate::Robot& robot)
{
    RobotPower = robot.getMotorState();
    return RobotPower;
};

//设置机械臂运行速度
bool HG_AI_Robot::Robot_Set_Speed(double New_Move_Speed, xmate::Robot& robot)
{  
  
  if(New_Move_Speed > 0.3){
      New_Move_Speed = 0.3;
  }
  Move_Speed = New_Move_Speed;  
  std::cout<<"Set Robot New Speed: "<<New_Move_Speed<<std::endl;
  return true;
};

//AR码信息更新
void HG_AI_Robot::Get_AR_Pose(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{  
   
    // Ar_Pose = {{0.0}};
    
    if (!msg->markers.empty()) {
        for(int i=0;i<msg->markers.size();i++){
                //  位置
                Ar_Pose[msg->markers[i].id][0] = msg->markers[i].pose.pose.position.x;
                Ar_Pose[msg->markers[i].id][1] = msg->markers[i].pose.pose.position.y;
                Ar_Pose[msg->markers[i].id][2] = msg->markers[i].pose.pose.position.z;
                //  四元数
                Ar_Pose[msg->markers[i].id][3] = msg->markers[i].pose.pose.orientation.w;
                Ar_Pose[msg->markers[i].id][4] = msg->markers[i].pose.pose.orientation.x;
                Ar_Pose[msg->markers[i].id][5] = msg->markers[i].pose.pose.orientation.y;
                Ar_Pose[msg->markers[i].id][6] = msg->markers[i].pose.pose.orientation.z;
        }
        //std::cout<<"current ar pose: "<<test<<std::endl;
	
    }
};

//UR码信息更新
void HG_AI_Robot::Get_UR_Pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{   
    Ur_Pose[0][0] = msg->pose.position.x;
    Ur_Pose[0][1] = msg->pose.position.y;
    Ur_Pose[0][2] = msg->pose.position.z;
    Ur_Pose[0][3] = msg->pose.orientation.w;
    Ur_Pose[0][4] = msg->pose.orientation.x;
    Ur_Pose[0][5] = msg->pose.orientation.y;
    Ur_Pose[0][6] = msg->pose.orientation.z;
};
void HG_AI_Robot::Get_UR_Pose2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{   
    Ur_Pose[1][0] = msg->pose.position.x;
    Ur_Pose[1][1] = msg->pose.position.y;
    Ur_Pose[1][2] = msg->pose.position.z;
    Ur_Pose[1][3] = msg->pose.orientation.w;
    Ur_Pose[1][4] = msg->pose.orientation.x;
    Ur_Pose[1][5] = msg->pose.orientation.y;
    Ur_Pose[1][6] = msg->pose.orientation.z;
};

void HG_AI_Robot::clean_ar_data()
{
    memset(Ar_Pose,0,sizeof(Ar_Pose));
    std::cout<<"Current ar-data: "<<Ar_Pose<<std::endl;
};

void HG_AI_Robot::clean_ur_data()
{
    memset(Ur_Pose,0,sizeof(Ur_Pose));
    std::cout<<"Current ur-data: "<<Ur_Pose<<std::endl;
};